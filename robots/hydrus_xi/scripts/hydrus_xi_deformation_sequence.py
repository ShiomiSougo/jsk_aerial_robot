#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（CasADi等式制約対応・完全版）
ゴール直前ソフトランディング減速＆静定ウェイト版
物理適応型安全装置付き：機体の物理状態（トルク）を監視し、安定している時のみ変形を進める

使用例:
  python hydrus_xi_deformation_sequence.py 0.0 -1.0 0.0
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum

class SequenceStep(Enum):
    INIT = 0                      # 初期ホバリング
    JOINT1_3_PRETENSION = 1       # ★統合追加: Joint 1 & 3 の同時予張力生成（機体構造の遊び除去と安定化）
    JOINT1_DEFORM = 2             # Joint 1 の純空力変形
    JOINT1_STABILIZE = 3          # Joint 1 変形後の機体揺れ収束待ち（待機フェーズ）
    JOINT3_DEFORM = 4             # Joint 3 の純空力変形 (順序繰り上げ)
    JOINT3_STABILIZE = 5          # Joint 3 変形後の機体揺れ収束待ち（待機フェーズ）
    JOINT2_SERVO = 6              # Joint 2 のサーボ変形（安全特異点スロープ徐変制御）
    COMPLETE = 7                  # 完了

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT_RAMP_RATE_BASE = 0.005     # 基本スロープ速度 [rad/loop]

# 静定判定用のパラメータ
STABILIZE_VELOCITY_THRESH = 0.01  # 静定したとみなす角速度の閾値 [rad/s]
STABILIZE_REQUIRED_LOOPS = 10     # 閾値を連続で下回るべきループ数 (10ループ = 約0.5秒)
STABILIZE_TIMEOUT = 4.0           # 揺れが収まらなくても次のステップへ進む最大制限時間 [s]

# ★追加: 物理的な安定判定・予張力用パラメータ
STABLE_TORQUE_THRESH = 0.05       # 安定と判定するトルク閾値 [Nm]
PRELOAD_TORQUE = 0.15             # 構造を突っ張らせるための予張力トルク [Nm]
JOINT2_HOLD_TORQUE = 0.08         # 予張力フェーズでJoint2の動きを抑え込むためのバイアストルク [Nm]

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,
    SequenceStep.JOINT1_3_PRETENSION: 2.0,  # 予張力にかける時間
}

LOOP_FREQ = 20.0                 # [Hz]
DT = 1.0 / LOOP_FREQ

class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        self.current_q = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_dq = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_effort = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        self.stabilize_loop_count = 0  # 静定確認用のカウンタ
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        rospy.loginfo("[HydrusXiSequencer] Initialized: q1=%.3f, q2=%.3f, q3=%.3f (Safety Mode Enabled)", target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)

    def update_target_angles(self, q1, q2, q3):
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        self.stabilize_loop_count = 0
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標 angles 受理。再始動。")

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_q:
                self.current_q[name] = msg.position[i]
                self.current_dq[name] = msg.velocity[i]
                if i < len(msg.effort):
                    self.current_effort[name] = msg.effort[i]

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _get_angle_difference(self, current, target):
        return self._normalize_angle(target - current)

    def _send_synchronized_command(self):
        """【排他制御・同期コマンド送信関数】"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        for joint_name in ['joint1', 'joint2', 'joint3']:
            msg.name.append(joint_name)
            
            # === ① Joint 1 の純空力変形中 ===
            if joint_name == 'joint1' and self.current_step == SequenceStep.JOINT1_DEFORM:
                msg.position.append(999.0)  # 位置PID遮断
                # ★修正: velocity は絶対に append しない（空配列のまま送る）
                msg.effort.append(0.0)      # ★修正: 変なダンピング抵抗も一旦やめ、純粋な空力だけで回す
                
            # === ② Joint 3 の純空力変形中 ===
            elif joint_name == 'joint3' and self.current_step == SequenceStep.JOINT3_DEFORM:
                msg.position.append(999.0)  # 位置PID遮断
                # ★修正: velocity は絶対に append しない
                msg.effort.append(0.0)
                
            # === ③ 予張力生成中、保持関節、および静定待機フェーズ ===
            else:
                msg.position.append(float(self.joint_targets[joint_name]))
                # ★修正: ここでも velocity は絶対に append しない
                
                # 予張力フェーズ中のみ、Joint 2 が動かないように徐々に保持トルクを加える
                effort_comp = 0.0
                if joint_name == 'joint2' and self.current_step == SequenceStep.JOINT1_3_PRETENSION:
                    elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
                    progress = min(1.0, elapsed / STEP_DURATIONS[SequenceStep.JOINT1_3_PRETENSION])
                    effort_comp = JOINT2_HOLD_TORQUE * progress
                    
                msg.effort.append(effort_comp)
                
        self.joints_ctrl_pub.publish(msg)

    def _send_internal_moment_command(self, joint_idx, tau_des):
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment(self, joint_name):
        """
        🛠️ 【全域一定速度＋特異点・ゴール直前ソフトランディング減速モデル】
        目標地点に近づくにつれてプロペラの風を自律的に弱め、衝突現象と逆戻りを防ぐ。
        """
        angle_diff_to_final = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        
        # 💡 修正：C++が等式制約で正確に応答するため、Pゲインと最大トルクを実際の物理スケールに最適化
        P_GAIN = 0.8# (元 0.8) 実トルクに直接反映されるためゲインを適正化
        MAX_DRIVE_TORQUE_BASE = 0.5  # (元 0.06) 目標とする最大発生トルク 0.3Nm を上限に設定
        
        # 1. 基礎となる目標モーメント命令値
        tau_des = P_GAIN * angle_diff_to_final
        
        # 2. 💡 ゴール手前での減速制御（フェードアウト処理）
        remaining_angle = abs(angle_diff_to_final)
        DECEL_ZONE = 0.15  # 減速を開始するゴール手前の残差エリア [rad] (約8.5度)
        
        if remaining_angle < DECEL_ZONE:
            # ゴールに近づくほど 1.0 ➔ 0.0 へと滑らかに絞り込まれる減速係数
            fade_factor = remaining_angle / DECEL_ZONE
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE * fade_factor
            rospy.logdebug(f"[{joint_name}] ゴール接近・減速中: 出力上限を {fade_factor*100:.1f}% に抑制")
        else:
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE
            
        # 3. 動的に計算された上限値でクリッピング（飽和処理）
        if tau_des > dynamic_max_torque:
            tau_des = dynamic_max_torque
        elif tau_des < -dynamic_max_torque:
            tau_des = -dynamic_max_torque
            
        return tau_des

    # ================= 安全装置付き目標更新関数 =================
    def _update_target_smoothly(self, joint_name):
        """
        💡 物理適応の肝: トルクが安定している時だけ目標を更新
        これにより発散を防止し、安定した変形を実現する
        """
        if abs(self.current_effort[joint_name]) < STABLE_TORQUE_THRESH:
            angle_diff = self._get_angle_difference(self.joint_targets[joint_name], self.target_q[joint_name])
            if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
                self.joint_targets[joint_name] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
            else:
                self.joint_targets[joint_name] = self.target_q[joint_name]

    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint2'] = self.current_q['joint2']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.INIT]:
            rospy.loginfo("[HydrusXiSequencer] Step 0 Completed -> Step 1 (Joint 1 & 3 Pretension)")
            self.current_step = SequenceStep.JOINT1_3_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joints_pretension(self):
        """★ 新規統合: Joint 1と3の両端から同時に予張力をかけ、構造全体の遊びを除去する"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        
        self._send_synchronized_command()
        
        # Joint 1 と 3 に対して逆向きのトルクをかけ、機体を突っ張らせる
        self._send_internal_moment_command(0, PRELOAD_TORQUE)
        self._send_internal_moment_command(2, -PRELOAD_TORQUE)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT1_3_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 1 Completed: 構造のプリロード完了 -> Step 2 (Joint 1 Deform)")
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        # ★ 物理適応型安全装置: トルク安定時のみ目標更新
        self._update_target_smoothly('joint1')
            
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 1 変形命令終了 -> Step 3 (機体静定待ちフェーズへ)")
            self._send_internal_moment_command(0, 0.0)
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT1_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint1_stabilize(self):
        """Joint 1 変形後の揺れを収束させる待機ルーチン"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        
        current_vel = abs(self.current_dq['joint1'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if current_vel < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else:
            self.stabilize_loop_count = 0
            
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS:
            rospy.loginfo("[HydrusXiSequencer] 🌊 Joint 1 の揺れが収束しました (待機時間: %.2f秒) -> Step 4 (Joint 3 Deform)")
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()
        elif duration >= STABILIZE_TIMEOUT:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ 静定待ちタイムアウト (4.0秒経過) 強制的にステップ4へ進みます。")
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        """Step 4: Joint 3 の純空力変形"""
        self.joint_targets['joint1'] = self.target_q['joint1']

        # ★ 物理適応型安全装置: トルク安定時のみ目標更新
        self._update_target_smoothly('joint3')
            
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 3 変形命令終了 -> Step 5 (機体静定待ちフェーズへ)")
            self._send_internal_moment_command(2, 0.0)
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT3_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint3_stabilize(self):
        """Joint 3 変形後の揺れを収束させる待機ルーチン"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        
        current_vel = abs(self.current_dq['joint3'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if current_vel < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else:
            self.stabilize_loop_count = 0
            
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS:
            rospy.loginfo("[HydrusXiSequencer] 🌊 Joint 3 の揺れが収束しました (待機時間: %.2f秒) -> Step 6 (Joint 2 Servo)")
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()
        elif duration >= STABILIZE_TIMEOUT:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ 静定待ちタイムアウト (4.0秒経過) 強制的にステップ6へ進みます。")
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()

    def _step_joint2_servo(self):
        """Step 6: Joint 2 のサーボ変形"""
        self.joint_targets['joint1'] = self.target_q['joint1']
        self.joint_targets['joint3'] = self.target_q['joint3']

        q1_abs = abs(self.current_q['joint1'])
        q3_abs = abs(self.current_q['joint3'])
        q2_abs = abs(self.current_q['joint2'])
       
        proximity_to_singularity = max(0.0, min(1.0, 1.0 - (q1_abs + q3_abs + q2_abs) / 2.0))
        
        ramp_reduction_factor = 1.0 - 0.9 * (proximity_to_singularity ** 2)
        dynamic_ramp_rate = JOINT_RAMP_RATE_BASE * ramp_reduction_factor
        
        angle_diff = self._get_angle_difference(self.joint_targets['joint2'], self.target_q['joint2'])
        if abs(angle_diff) > dynamic_ramp_rate:
            self.joint_targets['joint2'] += math.copysign(dynamic_ramp_rate, angle_diff)
        else:
            self.joint_targets['joint2'] = self.target_q['joint2']
            
        self._send_synchronized_command()
        
        tau_comp1 = self._calculate_target_moment('joint1')
        tau_comp3 = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(0, tau_comp1 * 0.5)
        self._send_internal_moment_command(2, tau_comp3 * 0.5)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Step 6 Completed -> Step 7 (COMPLETE)")
            self.current_step = SequenceStep.COMPLETE
            self.step_start_time = rospy.Time.now()

    def _step_complete(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 全シーケンス正常に完走しました！入力待機中...")

    def _control_loop(self, event):
        try:
            current_time = rospy.Time.now()
            if current_time.is_zero(): return
            if self.step_start_time is None: self.step_start_time = current_time
            
            if self.current_step == SequenceStep.INIT: self._step_init()
            elif self.current_step == SequenceStep.JOINT1_3_PRETENSION: self._step_joints_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT1_STABILIZE: self._step_joint1_stabilize()
            elif self.current_step == SequenceStep.JOINT3_DEFORM: self._step_joint3_deform()
            elif self.current_step == SequenceStep.JOINT3_STABILIZE: self._step_joint3_stabilize()
            elif self.current_step == SequenceStep.JOINT2_SERVO: self._step_joint2_servo()
            elif self.current_step == SequenceStep.COMPLETE: self._step_complete()
        except Exception as e:
            rospy.logerr("[HydrusXiSequencer] Loop Error: %s", str(e))

    def shutdown(self):
        self.loop_timer.shutdown()

def main():
    rospy.init_node('hydrus_xi_deformation_sequencer', log_level=rospy.INFO)
    target_q1, target_q2, target_q3 = 0.0, 0.0, 0.0
    if len(sys.argv) >= 4:
        target_q1, target_q2, target_q3 = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    
    sequencer = HydrusXiDeformationSequencer(target_q1, target_q2, target_q3)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        if sequencer.current_step == SequenceStep.COMPLETE:
            print("\n" + "="*60)
            print(" ✨ 【Hydrus-Xi】全シーケンス完走システム")
            print(" 次の目標関節角度 [q1 q2 q3] を入力してください。")
            print("="*60)
            try:
                user_input = input("💡 ターゲット入力 -> : ")
                if user_input.strip().lower() == 'q': break
                angles = [float(x) for x in user_input.split()]
                if len(angles) == 3: sequencer.update_target_angles(angles[0], angles[1], angles[2])
            except (ValueError, KeyboardInterrupt): break
        else:
            rate.sleep()
    sequencer.shutdown()

if __name__ == '__main__':
    main()