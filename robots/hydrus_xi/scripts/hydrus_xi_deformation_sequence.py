#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（サラサラURDF・安全ソフトランディング版）
変形順序変更版: Joint 1 (空力) ➔ Joint 2 & 3 (サーボ同時変形)

【修正1・最重要】センチネル値 target_joint_index=-1 でモーメント制御OFF
【修正4】single message for Joint2&3 Servo to avoid race condition

使用例:
  python hydrus_xi_deformation_sequence.py -0.3 1.0 -0.3
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from enum import Enum

class SequenceStep(Enum):
    INIT = 0
    JOINT1_PRETENSION = 1
    JOINT1_DEFORM = 2
    JOINT1_STABILIZE = 3
    JOINT2_3_SERVO = 4
    JOINT2_3_STABILIZE = 5
    COMPLETE = 6

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05
JOINT_RAMP_RATE_BASE = 0.005
STABILIZE_VELOCITY_THRESH = 0.01
STABILIZE_REQUIRED_LOOPS = 10
STABILIZE_TIMEOUT = 4.0
PRELOAD_TORQUE = 0.40

JOINT_CONTROLLERS = {
    'joint1': "/hydrus_xi/servo_controller/joints/controller1/simulation",
    'joint2': "/hydrus_xi/servo_controller/joints/controller2/simulation",
    'joint3': "/hydrus_xi/servo_controller/joints/controller3/simulation"
}

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,
    SequenceStep.JOINT1_PRETENSION: 2.0,
}

LOOP_FREQ = 20.0
DT = 1.0 / LOOP_FREQ

class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        self.all_joint_names = ['joint1', 'joint2', 'joint3']
        
        full_joints = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4', 'joint1', 'joint2', 'joint3']
        self.current_q = {name: 0.0 for name in full_joints}
        self.current_dq = {name: 0.0 for name in full_joints}
        self.current_effort = {name: 0.0 for name in full_joints}
        
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        self.stabilize_loop_count = 0
        
        rospy.wait_for_service('/hydrus_xi/controller_manager/switch_controller')
        self.switch_ctrl_client = rospy.ServiceProxy('/hydrus_xi/controller_manager/switch_controller', SwitchController)
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        rospy.loginfo("[HydrusXiSequencer] ⏳ 最初の /hydrus_xi/joint_states トピックの受信を待っています...")
        try:
            first_msg = rospy.wait_for_message('/hydrus_xi/joint_states', JointState, timeout=5.0)
            self._joint_state_callback(first_msg)
            
            self.joint_targets['joint1'] = self.current_q['joint1']
            self.joint_targets['joint2'] = self.current_q['joint2']
            self.joint_targets['joint3'] = self.current_q['joint3']
            
            rospy.loginfo("[HydrusXiSequencer] 🟩 初期状態の受信に成功。(q1=%.3f, q2=%.3f, q3=%.3f)", 
                         self.current_q['joint1'], self.current_q['joint2'], self.current_q['joint3'])
        except rospy.ROSException:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ トピック待機タイムアウト。初期値 0.0 で処理開始。")

        rospy.loginfo("[HydrusXiSequencer] ⏳ コントローラとの接続確立を待機...")
        rate_wait = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate_wait.sleep()
            
        self._send_synchronized_command()
        # ★ 【修正1・最重要】初期化時点でセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        rospy.loginfo("[HydrusXiSequencer] 🟩 コントローラ接続確立。初期姿勢+モーメント制御OFF送信。")

        rospy.loginfo("[HydrusXiSequencer] Initialized: q1_target=%.3f, q2_target=%.3f, q3_target=%.3f (Aerodynamic -> Servo Mode)", 
                     target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)
        
    def _switch_joint_controller(self, joint_key, action):
        """ROS Control の動的スイッチ"""
        controller_name = JOINT_CONTROLLERS[joint_key]
        try:
            req = SwitchControllerRequest()
            if action == 'start':
                req.start_controllers = [controller_name]
                req.stop_controllers = []
            elif action == 'stop':
                req.start_controllers = []
                req.stop_controllers = [controller_name]
            req.strictness = 1
            
            res = self.switch_ctrl_client(req)
            if res.ok:
                rospy.loginfo("[HydrusXiSequencer] 🛠️ コントローラ %s 成功: %s", action.upper(), controller_name)
                return True
            else:
                rospy.logerr("[HydrusXiSequencer] ❌ コントローラ %s 失敗: %s", action.upper(), controller_name)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return False

    def update_target_angles(self, q1, q2, q3):
        """
        ★ 【修正1・最重要】新規目標角度受信時に状態を完全リセット
        """
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        self.stabilize_loop_count = 0
        
        # ★ 【修正1】新規開始時もセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標受理: q1=%.3f, q2=%.3f, q3=%.3f。再始動。", q1, q2, q3)

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_q:
                self.current_q[name] = msg.position[i]
                self.current_dq[name] = msg.velocity[i]
                if i < len(msg.effort):
                    self.current_effort[name] = msg.effort[i]

    def _normalize_angle(self, angle):
        while angle > math.pi: 
            angle -= 2 * math.pi
        while angle < -math.pi: 
            angle += 2 * math.pi
        return angle

    def _get_angle_difference(self, current, target):
        return self._normalize_angle(target - current)

    def _send_synchronized_command(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        for name in self.all_joint_names:
            msg.name.append(name)
            if name in self.joint_targets:
                msg.position.append(float(self.joint_targets[name]))
            else:
                msg.position.append(float(self.current_q.get(name, 0.0)))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
                
        self.joints_ctrl_pub.publish(msg)

    def _send_internal_moment_command(self, joint_idx, tau_des):
        """
        ★ 【修正1・最重要】単一メッセージでモーメント制御のON/OFFを統一
        
        joint_idx = -1: モーメント制御OFF（C++側で has_moment_command_=false になる）
        joint_idx >= 0: モーメント制御ON（対象関節にトルクを指令）
        
        ★ 【修正4】毎ステップ単一メッセージのみ送信（queue_size=1 でのレース条件回避）
        """
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment_joint1(self):
        """
        Joint 1 の空力変形用モーメント計算
        符号付きトルク指令で、逆向き動作を防ぐ
        """
        angle_diff_to_final = self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])
        
        P_GAIN = 0.2
        MAX_DRIVE_TORQUE_BASE = 0.30
        MIN_FRICTION_TORQUE = 0.25

        tau_des = P_GAIN * angle_diff_to_final
        remaining_angle = abs(angle_diff_to_final)
        
        if remaining_angle > ANGLE_ERROR_THRESHOLD:
            if tau_des > 0 and tau_des < MIN_FRICTION_TORQUE:
                tau_des = MIN_FRICTION_TORQUE
            elif tau_des < 0 and tau_des > -MIN_FRICTION_TORQUE:
                tau_des = -MIN_FRICTION_TORQUE

        DECEL_ZONE = 0.05
        if remaining_angle < DECEL_ZONE:
            fade_factor = remaining_angle / DECEL_ZONE
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE * fade_factor
        else:
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE
            
        if tau_des > dynamic_max_torque:
            tau_des = dynamic_max_torque
        elif tau_des < -dynamic_max_torque:
            tau_des = -dynamic_max_torque
        
        if self.current_step == SequenceStep.JOINT1_DEFORM:
            rospy.loginfo_throttle(0.5, "[Joint1Deform] angle_diff=%.4f, tau_des=%.4f, remaining=%.4f", 
                                   angle_diff_to_final, tau_des, remaining_angle)
            
        return tau_des
    
    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        """初期ホバリング状態"""
        self._send_synchronized_command()
        # ★ 【修正1・最重要】INIT フェーズではセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        
        vel_sum = abs(self.current_dq['joint1']) + abs(self.current_dq['joint2']) + abs(self.current_dq['joint3'])
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= 5.0 and vel_sum < 0.005:
            rospy.loginfo("[HydrusXiSequencer] 初期静止完了 ➔ Step 1へ移行")
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()
        elif (rospy.Time.now() - self.step_start_time).to_sec() > 10.0:
            rospy.logwarn("[HydrusXiSequencer] 初期静止タイムアウト、強行移行")
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint1_pretension(self):
        """Joint1 のプリロード段階"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self._send_synchronized_command()
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        duration = STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]
        progress = min(1.0, elapsed / duration)
        
        current_preload = PRELOAD_TORQUE * progress
        # ★ 【修正1】target_joint_index=0 でモーメント制御ON
        self._send_internal_moment_command(0, current_preload)
        
        if elapsed >= duration:
            if self._switch_joint_controller('joint1', 'stop'):
                rospy.loginfo("[HydrusXiSequencer] Step 1 完了 ➔ Step 2 (Joint 1 純空力変形開始)")
                self.current_step = SequenceStep.JOINT1_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        """Joint1 を空力モーメントで変形（サーボ OFF）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self._send_synchronized_command()
        
        # ★ 【修正1】符号付きモーメント指令を毎ステップ送信（target_joint_index=0）
        tau_des = self._calculate_target_moment_joint1()
        self._send_internal_moment_command(0, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            # ★ 【修正1】モーメント制御OFF（センチネル値）
            self._send_internal_moment_command(-1, 0.0)
            self.joint_targets['joint1'] = self.current_q['joint1']
            self._send_synchronized_command()
            
            if self._switch_joint_controller('joint1', 'start'):
                rospy.loginfo("[HydrusXiSequencer] Joint 1 変形完了 ➔ Step 3 (Joint 1 静定待ち)")
                self.stabilize_loop_count = 0
                self.current_step = SequenceStep.JOINT1_STABILIZE
                self.step_start_time = rospy.Time.now()

    def _step_joint1_stabilize(self):
        """Joint1 変形後の静定待ち（サーボ ON）"""
        current_vel = abs(self.current_dq['joint1'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if duration < 0.5:
            self.joint_targets['joint1'] = self.current_q['joint1']
            
        self._send_synchronized_command()
        # ★ 【修正1・最重要】STABILIZE フェーズではセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        
        if duration >= 0.5:
            if current_vel < STABILIZE_VELOCITY_THRESH:
                self.stabilize_loop_count += 1
            else:
                self.stabilize_loop_count = 0
                
        MIN_WAIT = 5.0
        if duration >= MIN_WAIT:
            if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
                rospy.loginfo("[HydrusXiSequencer] Joint 1 静定完了 ➔ Step 4 (Joint 2 & 3 同時サーボ開始)")
                self.current_step = SequenceStep.JOINT2_3_SERVO
                self.step_start_time = rospy.Time.now()

    def _step_joint2_3_servo(self):
        """Joint 2 と 3 を同時にサーボ駆動"""
        q_sum = abs(self.current_q['joint1']) + abs(self.current_q['joint2']) + abs(self.current_q['joint3'])
        ramp_reduction_factor = max(0.2, 1.0 - 0.2 * q_sum)
        dynamic_ramp_rate = JOINT_RAMP_RATE_BASE * ramp_reduction_factor
        
        for joint in ['joint2', 'joint3']:
            angle_diff = self._get_angle_difference(self.joint_targets[joint], self.target_q[joint])
            if abs(angle_diff) > dynamic_ramp_rate:
                self.joint_targets[joint] += math.copysign(dynamic_ramp_rate, angle_diff)
            else:
                self.joint_targets[joint] = self.target_q[joint]
            
        self._send_synchronized_command()
        
        # ★ 【修正4・最重要】単一メッセージでセンチネル値（-1）を送信
        # queue_size=1 でのレース条件（idx0↔2切り替わり）を完全排除
        self._send_internal_moment_command(-1, 0.0)
        
        d2 = abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2']))
        d3 = abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3']))
        
        if d2 <= ANGLE_ERROR_THRESHOLD and d3 <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 2 & 3 同時サーボ駆動完了 ➔ Step 5 (静定待ち)")
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT2_3_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint2_3_stabilize(self):
        """両関節の静定待ち"""
        self._send_synchronized_command()
        # ★ 【修正1・最重要】STABILIZE フェーズではセンチネル値（-1）でモーメント制御OFF（単一メッセージ）
        self._send_internal_moment_command(-1, 0.0)

        duration = (rospy.Time.now() - self.step_start_time).to_sec()

        if abs(self.current_dq['joint2']) < STABILIZE_VELOCITY_THRESH and \
           abs(self.current_dq['joint3']) < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else:
            self.stabilize_loop_count = 0

        MIN_WAIT = 3.0
        if duration >= MIN_WAIT:
            if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
                rospy.loginfo("[HydrusXiSequencer] 機体全体の静定完了 ➔ Step 6 (シーケンス完了)")
                self.current_step = SequenceStep.COMPLETE
                self.step_start_time = rospy.Time.now()

    def _step_complete(self):
        """シーケンス完了"""
        self._send_synchronized_command()
        # ★ 【修正1・最重要】COMPLETE フェーズではセンチネル値（-1）でモーメント制御OFF（単一メッセージ）
        self._send_internal_moment_command(-1, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 全変形シーケンス完走！")

    def _control_loop(self, event):
        """メイン制御ループ"""
        try:
            current_time = rospy.Time.now()
            if current_time.is_zero(): 
                return
            if self.step_start_time is None: 
                self.step_start_time = current_time
            
            if self.current_step == SequenceStep.INIT: 
                self._step_init()
            elif self.current_step == SequenceStep.JOINT1_PRETENSION: 
                self._step_joint1_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: 
                self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT1_STABILIZE: 
                self._step_joint1_stabilize()
            elif self.current_step == SequenceStep.JOINT2_3_SERVO: 
                self._step_joint2_3_servo()
            elif self.current_step == SequenceStep.JOINT2_3_STABILIZE: 
                self._step_joint2_3_stabilize()
            elif self.current_step == SequenceStep.COMPLETE: 
                self._step_complete()
        except Exception as e:
            rospy.logerr("[HydrusXiSequencer] Loop Error: %s", str(e))

    def shutdown(self):
        """シャットダウン処理"""
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
            print(" ✨ 【Hydrus-Xi】ROS Control 動的スイッチ変形システム")
            print(" 次の目標関節角度 [q1 q2 q3] を入力してください。")
            print("="*60)
            try:
                user_input = input("💡 ターゲット入力 -> : ")
                if user_input.strip().lower() == 'q': 
                    break
                angles = [float(x) for x in user_input.split()]
                if len(angles) == 3: 
                    sequencer.update_target_angles(angles[0], angles[1], angles[2])
            except (ValueError, KeyboardInterrupt, EOFError): 
                break
        else:
            rate.sleep()
    sequencer.shutdown()

if __name__ == '__main__':
    main()