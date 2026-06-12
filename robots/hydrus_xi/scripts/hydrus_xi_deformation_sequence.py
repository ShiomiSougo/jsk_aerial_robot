#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（Gazeboコントローラ切り替え型・純トルク制御一本化版）

使用例:
  python hydrus_xi_deformation_sequence.py 0.8 -1.0 0.9
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController  # コントローラ切り替え用
from enum import Enum

class SequenceStep(Enum):
    INIT = 0                      # 初期ホバリング
    JOINT1_PRETENSION = 1         # Joint 1 の予張力生成
    JOINT1_DEFORM = 2             # Joint 1 の協調空力変形（コントローラOFF・0.05Nm純トルク）
    JOINT3_PRETENSION = 3         # Joint 3 の予張力生成
    JOINT3_DEFORM = 4             # Joint 3 の協調空力変形（コントローラOFF・0.05Nm純トルク）
    JOINT2_SERVO = 5              # Joint 2 のサーボ変形（一直線大暴れ自律回避スロープ）
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
        
        # === Gazeboのコントローラ切り替えサービスの初期化 ===
        rospy.loginfo("[HydrusXiSequencer] Waiting for Gazebo controller_manager service...")
        rospy.wait_for_service('/hydrus_xi/controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy('/hydrus_xi/controller_manager/switch_controller', SwitchController)
        rospy.loginfo("[HydrusXiSequencer] Gazebo controller_manager connected!")
        
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

    # === GazeboのコントローラON/OFF関数 ===
    def _set_gazebo_controller(self, joint_name, enable):
        """ Gazeboの特定関節の位置コントローラをON(固定) / OFF(脱力) する """
        ctrl_num = joint_name.replace('joint', '')
        controller_name = f'/hydrus_xi/servo_controller/joints/controller{ctrl_num}/simulation'
        
        start_ctrl = [controller_name] if enable else []
        stop_ctrl = [] if enable else [controller_name]
        
        try:
            resp = self.switch_controller(start_controllers=start_ctrl, stop_controllers=stop_ctrl, strictness=1)
            state_str = "STARTED (Locked)" if enable else "STOPPED (Free)"
            if resp.ok:
                rospy.loginfo(f"[HydrusXiSequencer] {joint_name} Controller successfully {state_str}.")
            else:
                rospy.logwarn(f"[HydrusXiSequencer] Failed to switch {joint_name} Controller.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _send_synchronized_command(self):
        """
        🛠️ 【排他制御対応型・純トルク流し込み同期関数】
        コントローラマネージャによって位置PIDがOFFにされている関節に対し、
        回転方向と真逆に一定 0.05 N·m の純トルクをダイレクトにインジェクションします。
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        for joint_name in ['joint1', 'joint2', 'joint3']:
            msg.name.append(joint_name)
            msg.position.append(float(self.joint_targets[joint_name]))
            msg.velocity.append(0.0)
            
            # --- ① Joint 1 の純空力変形中（コントローラOFF時）---
            if joint_name == 'joint1' and self.current_step == SequenceStep.JOINT1_DEFORM:
                dq = self.current_dq['joint1']
                torque_cmd = -math.copysign(0.05, dq) if abs(dq) > 0.005 else 0.0
                msg.effort.append(torque_cmd)
                
            # --- ② Joint 3 の純空力変形中（コントローラOFF時）---
            elif joint_name == 'joint3' and self.current_step == SequenceStep.JOINT3_DEFORM:
                # 💡【方向補正】対称性を考慮し、実測角速度 dq の【逆向き】に 0.05 Nm を出力
                dq = self.current_dq['joint3']
                torque_cmd = -math.copysign(0.05, dq) if abs(dq) > 0.005 else 0.0
                msg.effort.append(torque_cmd)
                
            # --- ③ それ以外の通常保持状態の関節 ---
            else:
                msg.effort.append(0.0)
                
        self.joints_ctrl_pub.publish(msg)

    def _send_internal_moment_command(self, joint_idx, tau_des):
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment(self, joint_name):
        """ 【非線形形状変化対応型・動的ゲインブーストモデル】 """
        angle_diff_to_final = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        
        P_GAIN_BASE = 1.8
        MIN_DRIVE_TORQUE_BASE = 0.40
        
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

    # ================= 各ステップ実行関数 =================

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
        """Step 1: Joint 1 の予張力生成（Joint 3 は位置固定して垂れ下がりを完璧に防止）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 1 Completed -> Step 2")
            
            # === 変形開始時にコントローラをOFFにして位置PIDを完全に遮断（脱力）===
            self._set_gazebo_controller('joint1', False)
            
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        """Step 2: Joint 1 純空力変形フェーズ"""
        angle_diff = self._get_angle_difference(self.joint_targets['joint1'], self.target_q['joint1'])
        if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
            self.joint_targets['joint1'] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
        else:
            self.joint_targets['joint1'] = self.target_q['joint1']
            
        self._send_synchronized_command()  # ここで自動的に0.05Nmの逆トルクが印加されます
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        # === 完了判定を「実際の物理角度(current_q)」が目標値に到達したかに変更 ===
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 1 Deformation Completed -> Step 3")
            
            # 変形完了時に指令値を最終ゴールに完全同期させ、コントローラを再起動してガチッと「固定」
            self.joint_targets['joint1'] = self.target_q['joint1']
            self._set_gazebo_controller('joint1', True)
            
            self._send_internal_moment_command(0, 0.0)
            self.current_step = SequenceStep.JOINT3_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint3_pretension(self):
        """Step 3: Joint 3 の予張力生成（変形を終えた Joint 1 は位置固定して形状を死守）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        
        self._send_synchronized_command()
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT3_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 3 Completed -> Step 4")
            
            # === 変形開始時にコントローラをOFFにして位置PIDを完全に遮断（脱力）===
            self._set_gazebo_controller('joint3', False)
            
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        """Step 4: Joint 3 純空力変形フェーズ"""
        angle_diff = self._get_angle_difference(self.joint_targets['joint3'], self.target_q['joint3'])
        if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
            self.joint_targets['joint3'] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
        else:
            self.joint_targets['joint3'] = self.target_q['joint3']
            
        self._send_synchronized_command()  # ここで自動的に0.05Nmの逆トルク（鏡像補正版）が印加されます
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        # === 完了判定を「実際の物理角度(current_q)」が目標値に到達したかに変更 ===
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 3 Deformation Completed -> Step 5")
            
            # 変形完了時にコントローラを再起動してガチッと「固定」
            self.joint_targets['joint3'] = self.target_q['joint3']
            self._set_gazebo_controller('joint3', True)
            
            self._send_internal_moment_command(2, 0.0)
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()

    def _step_joint2_servo(self):
        """
        Step 5: Joint 2 サーボ変形フェーズ
        🛠️ 【一直線形状移行時のLQI大暴れ・不安定化を完全にねじ伏せる徐変ロジック】
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