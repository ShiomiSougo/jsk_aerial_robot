#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（空力変形スロープ追従版）
C++側の最適化負荷を激減させるため、空力変形中も目標角度を細かく刻んで（ランプ状に）与えます。
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
    INIT = 0                      # 初期ホバリング
    JOINT1_3_PRETENSION = 1       # Joint 1 のプリロード
    JOINT1_DEFORM = 2             # Joint 1 の純空力変形（★刻み追従）
    JOINT1_STABILIZE = 3          # Joint 1 変形後の機体揺れ収束待ち
    JOINT3_DEFORM = 4             # Joint 3 の純空力変形（★刻み追従）
    JOINT3_STABILIZE = 5          # Joint 3 変形後の機体揺れ収束待ち
    JOINT2_SERVO = 6              # Joint 2 のサーボ変形
    COMPLETE = 7                  # 完了

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]

# 💡 探索をスムーズにするための刻み速度 [rad/loop] 
# (20Hzなので、0.01だと毎秒 0.2 rad = 約11度ずつ滑らかに変形します)
JOINT_RAMP_RATE_BASE = 0.01     

# 静定判定用のパラメータ
STABILIZE_VELOCITY_THRESH = 0.01  # 静定角速度閾値 [rad/s]
STABILIZE_REQUIRED_LOOPS = 10     # 収束ループ数
STABILIZE_TIMEOUT = 4.0           # タイムアウト時間 [s]

# 物理的な予張力パラメータ
PRELOAD_TORQUE = 0.40             # プリロードトルク [Nm]

JOINT_CONTROLLERS = {
    'joint1': "/hydrus_xi/servo_controller/joints/controller1/simulation",
    'joint2': "/hydrus_xi/servo_controller/joints/controller2/simulation",
    'joint3': "/hydrus_xi/servo_controller/joints/controller3/simulation"
}

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,
    SequenceStep.JOINT1_3_PRETENSION: 2.0,
}

LOOP_FREQ = 20.0                 # [Hz]
DT = 1.0 / LOOP_FREQ

class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        
        # 💡 追加：最終目標へ向けて少しずつ刻むための「現在の仮想目標角度」
        self.ramp_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.all_joint_names = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4', 'joint1', 'joint2', 'joint3']
        self.current_q = {name: 0.0 for name in self.all_joint_names}
        self.current_dq = {name: 0.0 for name in self.all_joint_names}
        self.current_effort = {name: 0.0 for name in self.all_joint_names}
        
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
            
            # 仮想目標の初期値を現在の状態に同期
            self.ramp_targets['joint1'] = self.current_q['joint1']
            self.ramp_targets['joint2'] = self.current_q['joint2']
            self.ramp_targets['joint3'] = self.current_q['joint3']
            
            rospy.loginfo("[HydrusXiSequencer] 🟩 初期状態の受信に成功しました。")
        except rospy.ROSException:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ トピックの待機がタイムアウトしました。")

        rate_wait = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate_wait.sleep()
            
        self._send_synchronized_command()
        rospy.loginfo("[HydrusXiSequencer] 🟩 コントローラとの接続が確立。")
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)
        
    def _switch_joint_controller(self, joint_key, action):
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
            return res.ok
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return False

    def update_target_angles(self, q1, q2, q3):
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        
        # 変形開始時は、現在の角度から滑らかにスタートするように追従の起点をリセット
        self.ramp_targets['joint1'] = self.current_q['joint1']
        self.ramp_targets['joint2'] = self.current_q['joint2']
        self.ramp_targets['joint3'] = self.current_q['joint3']
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        self.stabilize_loop_count = 0
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標 angles 受理。スロープ追従を開始します。")

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

    def _update_ramp_target(self, joint_name, rate=JOINT_RAMP_RATE_BASE):
        """💡 追加：最終目標角度に向けて、仮想目標角度を毎ループ少しずつ刻む関数"""
        angle_diff = self._get_angle_difference(self.ramp_targets[joint_name], self.target_q[joint_name])
        if abs(angle_diff) > rate:
            self.ramp_targets[joint_name] += math.copysign(rate, angle_diff)
        else:
            self.ramp_targets[joint_name] = self.target_q[joint_name]

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
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment(self, joint_name):
        # 💡 修正：最終目標(target_q)ではなく、刻まれた仮想目標(ramp_targets)との偏差を使用！
        angle_diff_to_ramp = self._get_angle_difference(self.current_q[joint_name], self.ramp_targets[joint_name])
        
        P_GAIN = 1.2  # 刻み制御になったため、ゲインを少し高め（0.7 -> 1.2）にして追従性を向上
        MAX_DRIVE_TORQUE_BASE = 0.5
        tau_des = P_GAIN * angle_diff_to_ramp
        remaining_angle = abs(angle_diff_to_ramp)
        
        DECEL_ZONE = 0.02 
        if remaining_angle < DECEL_ZONE:
            fade_factor = remaining_angle / DECEL_ZONE
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE * fade_factor
        else:
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE
            
        if tau_des > dynamic_max_torque: tau_des = dynamic_max_torque
        elif tau_des < -dynamic_max_torque: tau_des = -dynamic_max_torque
            
        return tau_des
    
    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint2'] = self.current_q['joint2']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        
        vel_sum = abs(self.current_dq['joint1']) + abs(self.current_dq['joint2']) + abs(self.current_dq['joint3'])
        if (rospy.Time.now() - self.step_start_time).to_sec() >= 2.0 and vel_sum < 0.005:
            self.current_step = SequenceStep.JOINT1_3_PRETENSION  
            self.step_start_time = rospy.Time.now()

    def _step_joints_pretension(self):
        self.joint_targets['joint1'] = self.current_q['joint1']
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        duration = STEP_DURATIONS[SequenceStep.JOINT1_3_PRETENSION]
        progress = min(1.0, elapsed / duration)
        
        current_preload = PRELOAD_TORQUE * progress
        self._send_internal_moment_command(0, current_preload)
        
        if elapsed >= duration:
            if self._switch_joint_controller('joint1', 'stop'):
                rospy.loginfo("[HydrusXiSequencer] ➔ Step 2 (Joint 1 純空力変形・スロープ追従開始)")
                self.current_step = SequenceStep.JOINT1_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        # 💡 仮想目標を1刻み進める
        self._update_ramp_target('joint1')
        
        self.joint_targets['joint1'] = self.current_q['joint1'] 
        self._send_synchronized_command()
        
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        # 判定は「最終目標」に到達したかどうか
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(0, 0.0)
            self.joint_targets['joint1'] = self.current_q['joint1']
            if self._switch_joint_controller('joint1', 'start'):
                rospy.loginfo("[HydrusXiSequencer] Joint 1 変形完了 ➔ Step 3 (静定待ち)")
                self.stabilize_loop_count = 0
                self.current_step = SequenceStep.JOINT1_STABILIZE
                self.step_start_time = rospy.Time.now()

    def _step_joint1_stabilize(self):
        self._send_synchronized_command()
        current_vel = abs(self.current_dq['joint1'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if current_vel < STABILIZE_VELOCITY_THRESH: self.stabilize_loop_count += 1
        else: self.stabilize_loop_count = 0
            
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
            if self._switch_joint_controller('joint3', 'stop'):
                rospy.loginfo("[HydrusXiSequencer] ➔ Step 4 (Joint 3 純空力変形・スロープ追従開始)")
                self.current_step = SequenceStep.JOINT3_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        # 💡 仮想目標を1刻み進める
        self._update_ramp_target('joint3')
        
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(2, 0.0)
            self.joint_targets['joint3'] = self.current_q['joint3']
            if self._switch_joint_controller('joint3', 'start'):
                rospy.loginfo("[HydrusXiSequencer] Joint 3 変形完了 ➔ Step 5 (静定待ち)")
                self.stabilize_loop_count = 0
                self.current_step = SequenceStep.JOINT3_STABILIZE
                self.step_start_time = rospy.Time.now()

    def _step_joint3_stabilize(self):
        self._send_synchronized_command()
        current_vel = abs(self.current_dq['joint3'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if current_vel < STABILIZE_VELOCITY_THRESH: self.stabilize_loop_count += 1
        else: self.stabilize_loop_count = 0
            
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
            rospy.loginfo("[HydrusXiSequencer] ➔ Step 6 (Joint 2 Servo開始)")
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()

    def _step_joint2_servo(self):
        q1_abs = abs(self.current_q['joint1'])
        q3_abs = abs(self.current_q['joint3'])
        q2_abs = abs(self.current_q['joint2'])
       
        proximity_to_singularity = max(0.0, min(1.0, 1.0 - (q1_abs + q3_abs + q2_abs) / 2.0))
        ramp_reduction_factor = 1.0 - 0.4 * (proximity_to_singularity ** 2)
        dynamic_ramp_rate = JOINT_RAMP_RATE_BASE * ramp_reduction_factor
        
        angle_diff = self._get_angle_difference(self.joint_targets['joint2'], self.target_q['joint2'])
        if abs(angle_diff) > dynamic_ramp_rate:
            self.joint_targets['joint2'] += math.copysign(dynamic_ramp_rate, angle_diff)
        else:
            self.joint_targets['joint2'] = self.target_q['joint2']
            
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            self.current_step = SequenceStep.COMPLETE
            self.step_start_time = rospy.Time.now()

    def _step_complete(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 スロープ変形シーケンスが完走しました！")

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
            print(" ✨ 【Hydrus-Xi】空力変形スロープ追従システム")
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