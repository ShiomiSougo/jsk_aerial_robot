#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（完全版）
トルク出力のチャタリング対策：ローパスフィルタ＋デッドバンド実装済み
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum

class SequenceStep(Enum):
    INIT = 0                      # 初期ホバリング
    JOINT1_PRETENSION = 1         # Joint 1 の予張力生成
    JOINT1_DEFORM = 2             # Joint 1 の純空力変形
    JOINT1_STABILIZE = 3          # Joint 1 変形後の静定待ち
    JOINT3_PRETENSION = 4         # Joint 3 の予張力生成
    JOINT3_DEFORM = 5             # Joint 3 の純空力変形
    JOINT3_STABILIZE = 6          # Joint 3 変形後の静定待ち
    JOINT2_SERVO = 7              # Joint 2 のサーボ変形
    COMPLETE = 8                  # 完了

# --- 各種パラメータ ---
ANGLE_ERROR_THRESHOLD = 0.05
JOINT_RAMP_RATE_BASE = 0.005
STABILIZE_VELOCITY_THRESH = 0.01
STABILIZE_REQUIRED_LOOPS = 10
STABILIZE_TIMEOUT = 4.0
STABLE_TORQUE_THRESH = 0.05

# --- 制御用定数 ---
P_GAIN = 0.15
MAX_DRIVE_TORQUE_BASE = 0.3
TAU_LPF_ALPHA = 0.2    # ローパスフィルタ係数 (0.0 < alpha <= 1.0)
TAU_DEADBAND = 0.02    # トルクのデッドバンド [Nm]

LOOP_FREQ = 20.0
DT = 1.0 / LOOP_FREQ

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,
    SequenceStep.JOINT1_PRETENSION: 2.0,
    SequenceStep.JOINT3_PRETENSION: 2.0,
}

class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        self.current_q = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_dq = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_effort = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        # フィルタ用バッファ
        self.filtered_tau = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        self.stabilize_loop_count = 0
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        rospy.loginfo("[HydrusXiSequencer] Initialized with LPF/Deadband enabled.")
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)

    def update_target_angles(self, q1, q2, q3):
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        self.stabilize_loop_count = 0
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標受理。")

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_q:
                self.current_q[name] = msg.position[i]
                self.current_dq[name] = msg.velocity[i]
                if i < len(msg.effort):
                    self.current_effort[name] = msg.effort[i]

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _get_angle_difference(self, current, target):
        return self._normalize_angle(target - current)

    def _apply_lpf_and_deadband(self, joint_name, raw_tau):
        """ローパスフィルタとデッドバンドを適用"""
        self.filtered_tau[joint_name] = (1.0 - TAU_LPF_ALPHA) * self.filtered_tau[joint_name] + TAU_LPF_ALPHA * raw_tau
        if abs(self.filtered_tau[joint_name]) < TAU_DEADBAND:
            return 0.0
        return self.filtered_tau[joint_name]

    def _send_synchronized_command(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for joint_name in ['joint1', 'joint2', 'joint3']:
            msg.name.append(joint_name)
            if (self.current_step == SequenceStep.JOINT1_DEFORM and joint_name == 'joint1') or \
               (self.current_step == SequenceStep.JOINT3_DEFORM and joint_name == 'joint3'):
                msg.position.append(999.0)
                msg.velocity.append(0.0)
                dq = self.current_dq[joint_name]
                torque_cmd = -math.copysign(0.1, dq) if abs(dq) > 0.005 else 0.0
                msg.effort.append(torque_cmd)
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
        angle_diff = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        tau_des = P_GAIN * angle_diff
        
        # 減速処理
        remaining = abs(angle_diff)
        DECEL_ZONE = 0.15
        fade = min(1.0, remaining / DECEL_ZONE)
        dynamic_max = MAX_DRIVE_TORQUE_BASE * fade
        tau_des = max(min(tau_des, dynamic_max), -dynamic_max)
        
        return self._apply_lpf_and_deadband(joint_name, tau_des)

    def _update_target_smoothly(self, joint_name):
        if abs(self.current_effort[joint_name]) < STABLE_TORQUE_THRESH:
            angle_diff = self._get_angle_difference(self.joint_targets[joint_name], self.target_q[joint_name])
            if abs(angle_diff) > JOINT_RAMP_RATE_BASE:
                self.joint_targets[joint_name] += math.copysign(JOINT_RAMP_RATE_BASE, angle_diff)
            else:
                self.joint_targets[joint_name] = self.target_q[joint_name]

    # --- ステップ処理 ---
    def _step_init(self):
        self.joint_targets = self.current_q.copy()
        self._send_synchronized_command()
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.INIT]:
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint1_pretension(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, self._calculate_target_moment('joint1'))
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]:
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        self._update_target_smoothly('joint1')
        self._send_synchronized_command()
        self._send_internal_moment_command(0, self._calculate_target_moment('joint1'))
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(0, 0.0)
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT1_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint1_stabilize(self):
        self._send_synchronized_command()
        if abs(self.current_dq['joint1']) < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else: self.stabilize_loop_count = 0
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or (rospy.Time.now() - self.step_start_time).to_sec() >= STABILIZE_TIMEOUT:
            self.current_step = SequenceStep.JOINT3_PRETENSION
            self.step_start_time = rospy.Time.now()

    def _step_joint3_pretension(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(2, self._calculate_target_moment('joint3'))
        if (rospy.Time.now() - self.step_start_time).to_sec() >= STEP_DURATIONS[SequenceStep.JOINT3_PRETENSION]:
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        self._update_target_smoothly('joint3')
        self._send_synchronized_command()
        self._send_internal_moment_command(2, self._calculate_target_moment('joint3'))
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(2, 0.0)
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT3_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint3_stabilize(self):
        self._send_synchronized_command()
        if abs(self.current_dq['joint3']) < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else: self.stabilize_loop_count = 0
        if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or (rospy.Time.now() - self.step_start_time).to_sec() >= STABILIZE_TIMEOUT:
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()

    def _step_joint2_servo(self):
        angle_diff = self._get_angle_difference(self.joint_targets['joint2'], self.target_q['joint2'])
        self.joint_targets['joint2'] += math.copysign(min(abs(angle_diff), JOINT_RAMP_RATE_BASE), angle_diff)
        self._send_synchronized_command()
        
        # Joint 2 サーボ中も補正トルクを送る
        self._send_internal_moment_command(0, self._calculate_target_moment('joint1') * 0.5)
        self._send_internal_moment_command(2, self._calculate_target_moment('joint3') * 0.5)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            self.current_step = SequenceStep.COMPLETE

    def _step_complete(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)

    def _control_loop(self, event):
        if self.step_start_time is None: self.step_start_time = rospy.Time.now()
        steps = {
            SequenceStep.INIT: self._step_init,
            SequenceStep.JOINT1_PRETENSION: self._step_joint1_pretension,
            SequenceStep.JOINT1_DEFORM: self._step_joint1_deform,
            SequenceStep.JOINT1_STABILIZE: self._step_joint1_stabilize,
            SequenceStep.JOINT3_PRETENSION: self._step_joint3_pretension,
            SequenceStep.JOINT3_DEFORM: self._step_joint3_deform,
            SequenceStep.JOINT3_STABILIZE: self._step_joint3_stabilize,
            SequenceStep.JOINT2_SERVO: self._step_joint2_servo,
            SequenceStep.COMPLETE: self._step_complete
        }
        steps[self.current_step]()

    def shutdown(self):
        self.loop_timer.shutdown()

def main():
    rospy.init_node('hydrus_xi_deformation_sequencer')
    sequencer = HydrusXiDeformationSequencer(0.0, 0.0, 0.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if sequencer.current_step == SequenceStep.COMPLETE:
            try:
                user_input = input("💡 次の目標角度 [q1 q2 q3] を入力 (qで終了): ")
                if user_input.strip().lower() == 'q': break
                angles = [float(x) for x in user_input.split()]
                if len(angles) == 3: sequencer.update_target_angles(*angles)
            except (ValueError, EOFError): break
        rate.sleep()
    sequencer.shutdown()

if __name__ == '__main__':
    main()