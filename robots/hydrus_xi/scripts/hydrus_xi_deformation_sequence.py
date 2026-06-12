#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（純トルク制御一本化＆Joint3方向補正版）

使用例:
  python hydrus_xi_deformation_sequence.py 0.8 -1.0 0.9
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
    JOINT1_PRETENSION = 1         # Joint 1 の予張力生成（Joint 3は現在地で位置固定）
    JOINT1_DEFORM = 2             # Joint 1 の純空力変形（位置PID完全遮断・0.05Nm逆トルク固定）
    JOINT3_PRETENSION = 3         # Joint 3 の予張力生成（Joint 1は現在地で位置固定）
    JOINT3_DEFORM = 4             # Joint 3 の純空力変形（位置PID完全遮断・0.05Nm逆トルク固定）
    JOINT2_SERVO = 5              # Joint 2 のサーボ変形（安全特異点スロープ徐変制御）
    COMPLETE = 6                  # 完了

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT_RAMP_RATE_BASE = 0.001     # 基本スロープ速度 [rad/loop]

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,
    SequenceStep.JOINT1_PRETENSION: 2.0,
    SequenceStep.JOINT3_PRETENSION: 2.0,
}

LOOP_FREQ = 20.0                 # [Hz]
DT = 1.0 / LOOP_FREQ

class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        self.current_q = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_dq = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        rospy.loginfo("[HydrusXiSequencer] Initialized: q1=%.3f, q2=%.3f, q3=%.3f", target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)

    def update_target_angles(self, q1, q2, q3):
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標 angles 受理。再始動。")

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_q:
                self.current_q[name] = msg.position[i]
                self.current_dq[name] = msg.velocity[i]

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _get_angle_difference(self, current, target):
        return self._normalize_angle(target - current)

    def _send_synchronized_command(self):
        """
        🛠️ 【位置・トルク排他制御連動・同期コマンド送信関数】
        変形中の関節から位置制御を100%排除し、回転方向と逆向きに一定0.05Nmを出力する。
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        for joint_name in ['joint1', 'joint2', 'joint3']:
            msg.name.append(joint_name)
            
            # === ① Joint 1 の純空力変形中（Step 2）===
            if joint_name == 'joint1' and self.current_step == SequenceStep.JOINT1_DEFORM:
                msg.position.append(999.0)  # C++側の位置制御PIDを完全に遮断（無効化）する特殊フラグ
                msg.velocity.append(0.0)
                
                # 回転方向（dq）と【逆向き】に一定 0.05 N·m のブレーキ抵抗トルクを出力
                dq = self.current_dq['joint1']
                torque_cmd = -math.copysign(0.05, dq) if abs(dq) > 0.005 else 0.0
                msg.effort.append(torque_cmd)
                
            # === ② Joint 3 の純空力変形中（Step 4）===
            elif joint_name == 'joint3' and self.current_step == SequenceStep.JOINT3_DEFORM:
                msg.position.append(999.0)  # C++側の位置制御PIDを完全に遮断（無効化）する特殊フラグ
                msg.velocity.append(0.0)
                
                # 💡【方向修正】軸の定義が逆向きであることを考慮し、
                # 挙動が対照的になるよう、回転方向（dq）の【逆向き】に一定 0.05 N·m のトルクを出力
                dq = self.current_dq['joint3']
                torque_cmd = -math.copysign(0.05, dq) if abs(dq) > 0.005 else 0.0
                msg.effort.append(torque_cmd)
                
            # === ③ 予張力生成中、保持関節、および Joint 2 ===
            # トルク制御と位置制御が混ざらないよう、変形時以外はJsk標準の位置PID指令値（スロープ）で形状死守
            else:
                msg.position.append(float(self.joint_targets[joint_name]))
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
                
        self.joints_ctrl_pub.publish(msg)

    def _send_internal_moment_command(self, joint_idx, tau_des):
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment(self, joint_name):
        """【非線形形状変化対応型・動的ゲインブーストモデル】"""
        angle_diff_to_final = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        
        P_GAIN_BASE = 0.5
        MIN_DRIVE_TORQUE_BASE = 0.20
        
        init_diff = abs(self._get_angle_difference(self.joint_targets[joint_name], self.target_q[joint_name]))
        progress = 1.0
        if init_diff > 0.01:
            current_diff = abs(angle_diff_to_final)
            progress = max(0.0, min(1.0, 1.0 - (current_diff / init_diff)))
            
        boost_factor = 1.0 + 1.2 * (progress ** 2)
        
        P_GAIN = P_GAIN_BASE * boost_factor
        MIN_DRIVE_TORQUE = MIN_DRIVE_TORQUE_BASE * boost_factor
        
        tau_des = P_GAIN * angle_diff_to_final
        if abs(tau_des) < MIN_DRIVE_TORQUE and abs(angle_diff_to_final) > ANGLE_ERROR_THRESHOLD:
            tau_des = math.copysign(MIN_DRIVE_TORQUE, angle_diff_to_final)
            
        return tau_des

    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint2'] = self.current_q['joint2']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.INIT]:
            rospy.loginfo("[HydrusXiSequencer] Step 0 Completed -> Step 1")
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint1_pretension(self):
        """Step 1: Joint 1 の予張力生成（Joint 3 は現在地で通常位置PID保持し垂れ下がり防止）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 1 Completed -> Step 2")
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        """Step 2: Joint 1 の純空力変形（位置制御を完全に排除し、0.05Nmの逆トルクのみを印加）"""
        angle_diff = self._get_angle_difference(self.joint_targets['joint1'], self.target_q['joint1'])
        if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
            self.joint_targets['joint1'] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
        else:
            self.joint_targets['joint1'] = self.target_q['joint1']
            
        # 💡 ここで位置指令を999.0フラグ化し、純粋なトルク制御のみへと落とし込む
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        # 物理的な実測角度が目標値の閾値内に到達したことをもって完了判定とする（純トルク移行に伴う修正）
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 1 Deformation Completed -> Step 3")
            self._send_internal_moment_command(0, 0.0)
            self.current_step = SequenceStep.JOINT3_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint3_pretension(self):
        """Step 3: Joint 3 の予張力生成（変形完了した Joint 1 は現在地で位置固定して形状死守）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT3_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 3 Completed -> Step 4")
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        """Step 4: Joint 3 の純空力変形（位置制御を完全に排除し、0.05Nmの逆トルクのみを印加）"""
        angle_diff = self._get_angle_difference(self.joint_targets['joint3'], self.target_q['joint3'])
        if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
            self.joint_targets['joint3'] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
        else:
            self.joint_targets['joint3'] = self.target_q['joint3']
            
        # 💡 999.0フラグにより位置制御を排除、純粋なトルク制御のみに一本化
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 3 Deformation Completed -> Step 5")
            self._send_internal_moment_command(2, 0.0)
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()

    def _step_joint2_servo(self):
        """
        Step 5: Joint 2 サーボ変形フェーズ
        🛠️ 【一直線形状移行時のLQI不安定化を自律回避するロジック】
        """
        q1_abs = abs(self.current_q['joint1'])
        q3_abs = abs(self.current_q['joint3'])
        q2_abs = abs(self.current_q['joint2'])
        
        # 1. 各関節角度から「一直線特異点」への接近度を動的に計算 (0.0: 安全 〜 1.0: 危険)
        proximity_to_singularity = max(0.0, min(1.0, 1.0 - (q1_abs + q3_abs + q2_abs) / 2.0))
        
        # 2. 一直線に近づくほどスロープ変形速度を自動的に最大10分の1まで減速（発振をマイルドに抑制）
        ramp_reduction_factor = 1.0 - 0.9 * (proximity_to_singularity ** 2)
        dynamic_ramp_rate = JOINT_RAMP_RATE_BASE * ramp_reduction_factor
        
        angle_diff = self._get_angle_difference(self.joint_targets['joint2'], self.target_q['joint2'])
        if abs(angle_diff) > dynamic_ramp_rate:
            self.joint_targets['joint2'] += math.copysign(dynamic_ramp_rate, angle_diff)
        else:
            self.joint_targets['joint2'] = self.target_q['joint2']
            
        self._send_synchronized_command()
        
        # 3. 【能動的姿勢保持サポート】Joint 2 の変形に伴う反力をプロペラ側で打ち消すため、
        # すでに変形を終えてロックされている Joint 1 / 3 から「姿勢維持アシスト用の風」を継続して分散送信。
        tau_comp1 = self._calculate_target_moment('joint1')
        tau_comp3 = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(0, tau_comp1 * 0.5)
        self._send_internal_moment_command(2, tau_comp3 * 0.5)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Step 5 Completed -> Step 6 (COMPLETE)")
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
            elif self.current_step == SequenceStep.JOINT1_PRETENSION: self._step_joint1_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT3_PRETENSION: self._step_joint3_pretension()
            elif self.current_step == SequenceStep.JOINT3_DEFORM: self._step_joint3_deform()
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