#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 空力推力駆動 連続変形シーケンス実行スクリプト（制御分離・完全開通版）

使用例:
  python hydrus_xi_deformation_sequence.py 0.4 0.2 -0.4
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum

# ======================== 定数定義 ========================

class ControlState(Enum):
    """制御モードの定義"""
    LOCKED = 0      # 位置制御（剛体化）
    UNLOCKED = 1    # エフォート制御（摩擦シミュレーション）

class SequenceStep(Enum):
    """シーケンスのステップ定義"""
    INIT = 0                      # 初期ホバリング
    JOINT1_PRETENSION = 1         # Joint 1 の予張力生成
    JOINT1_DEFORM = 2             # Joint 1 の空力変形
    JOINT3_PRETENSION = 3         # Joint 3 の予張力生成
    JOINT3_DEFORM = 4             # Joint 3 の空力変形
    JOINT2_SERVO = 5              # Joint 2 のサーボ変形
    COMPLETE = 6                  # 完了

# パラメータ（調整可能）
FRICTION_COEFF = 0.01             # 摩擦係数 [N*m*s/rad]
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT2_RAMP_RATE = 0.01          # Joint 2 スロープ速度 [rad/loop]

STEP_DURATIONS = {
    SequenceStep.INIT: 2.0,                    # [秒] 初期待機
    SequenceStep.JOINT1_PRETENSION: 2.0,       # [秒] Joint 1 予張力
    SequenceStep.JOINT3_PRETENSION: 2.0,       # [秒] Joint 3 予張力
}

LOOP_FREQ = 20.0                 # メインループ周波数 [Hz]
DT = 1.0 / LOOP_FREQ             # ループ周期 [秒]

# ======================== ステートマシンクラス ========================

class HydrusXiDeformationSequencer:
    """
    Hydrus-Xi の変形シーケンスを制御するステートマシンクラス
    """
    
    def __init__(self, target_q1, target_q2, target_q3):
        """初期化"""
        # ===== ターゲット角度 =====
        self.target_q = {
            'joint1': target_q1,
            'joint2': target_q2,
            'joint3': target_q3
        }
        
        # ===== 現在の関節状態 =====
        self.current_q = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.current_dq = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        # ===== 制御モード =====
        self.control_mode = {
            'joint1': ControlState.LOCKED,
            'joint2': ControlState.LOCKED,
            'joint3': ControlState.LOCKED
        }
        
        # ===== シーケンス状態 =====
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        
        # ===== Joint 2 スロープ制御用 =====
        self.joint2_current_target = target_q2
        
        # ===== ROS パブリッシャ（正しい2本のトピックに分離） =====
        self.joints_ctrl_pub = rospy.Publisher(
            '/hydrus_xi/joints_ctrl',
            JointState,
            queue_size=1
        )
        
        self.joints_torque_ctrl_pub = rospy.Publisher(
            '/hydrus_xi/joints_torque_ctrl',
            JointState,
            queue_size=1
        )
        
        self.moment_pub = rospy.Publisher(
            '/hydrus_xi/target_internal_moment',
            Float64MultiArray,
            queue_size=1
        )
        
        # ===== ROS サブスクライバ =====
        self.joint_state_sub = rospy.Subscriber(
            '/hydrus_xi/joint_states',
            JointState,
            self._joint_state_callback
        )
        
        rospy.loginfo(
            "[HydrusXiSequencer] Initialized with target angles: "
            "q1=%.3f, q2=%.3f, q3=%.3f [rad]",
            target_q1, target_q2, target_q3
        )
        
        # ===== メインループタイマー =====
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)

    def update_target_angles(self, q1, q2, q3):
        """新しい目標角度を設定し、ステートマシンをリセット"""
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3
        
        self.control_mode['joint1'] = ControlState.LOCKED
        self.control_mode['joint2'] = ControlState.LOCKED
        self.control_mode['joint3'] = ControlState.LOCKED
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        
        rospy.loginfo(
            "[HydrusXiSequencer] 🔄 新しい目標角度を受理: "
            "q1=%.3f, q2=%.3f, q3=%.3f [rad]. シーケンスを再始動します。",
            q1, q2, q3
        )
    
    # ======================== コールバック関数 ========================
    
    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'joint1':
                self.current_q['joint1'] = msg.position[i]
                self.current_dq['joint1'] = msg.velocity[i]
            elif name == 'joint2':
                self.current_q['joint2'] = msg.position[i]
                self.current_dq['joint2'] = msg.velocity[i]
            elif name == 'joint3':
                self.current_q['joint3'] = msg.position[i]
                self.current_dq['joint3'] = msg.velocity[i]
    
    # ======================== ユーティリティ関数 ========================
    
    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle
    
    def _get_angle_difference(self, current, target):
        diff = target - current
        return self._normalize_angle(diff)
    
    def _set_control_mode(self, joint_name, mode):
        self.control_mode[joint_name] = mode
        rospy.loginfo("[HydrusXiSequencer] %s control mode: %s", joint_name, mode.name)
    
    def _send_position_command(self, joints_dict):
        """位置制御したい関節だけを明示的に指定して送信（これによって他が脱力できる）"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for joint_name in ['joint1', 'joint2', 'joint3']:
            if joint_name in joints_dict:
                msg.name.append(joint_name)
                msg.position.append(joints_dict[joint_name])
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        self.joints_ctrl_pub.publish(msg)
    
    def _send_effort_command(self, joints_dict):
        """トルク制御したい関節だけを明示的に指定して送信"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for joint_name in ['joint1', 'joint2', 'joint3']:
            if joint_name in joints_dict:
                msg.name.append(joint_name)
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(joints_dict[joint_name])
        self.joints_torque_ctrl_pub.publish(msg)
    
    def _send_internal_moment_command(self, joint_idx, tau_des):
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)
    
    def _calculate_friction_torque(self, joint_name):
        dq = self.current_dq[joint_name]
        return -FRICTION_COEFF * dq
    
    def _calculate_target_moment(self, joint_name):
        angle_diff = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        P_GAIN = 3.0  
        MIN_DRIVE_TORQUE = 0.5  
        
        tau_des = P_GAIN * angle_diff
        if abs(tau_des) < MIN_DRIVE_TORQUE and abs(angle_diff) > ANGLE_ERROR_THRESHOLD:
            tau_des = math.copysign(MIN_DRIVE_TORQUE, angle_diff)
        return tau_des
    
    # ======================== ステップ実行関数 ========================
    
    def _step_init(self):
        """Step 0: 初期ホバリング"""
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        if elapsed >= STEP_DURATIONS[SequenceStep.INIT]:
            rospy.loginfo("[HydrusXiSequencer] Step 0 (INIT) completed. Moving to Step 1.")
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()
    
    def _step_joint1_pretension(self):
        """Step 1: Joint 1 の予張力生成"""
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        if elapsed >= STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 1 (JOINT1_PRETENSION) completed. Moving to Step 2.")
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint1_deform(self):
        """Step 2: Joint 1 の空力変形（Joint1の位置固定を完全に排除）"""
        if self.control_mode['joint1'] != ControlState.UNLOCKED:
            self._set_control_mode('joint1', ControlState.UNLOCKED)
        
        tau_fric = self._calculate_friction_torque('joint1')
        
        # 修正：位置命令にはJoint1を含めず、完全にサーボの力を抜く
        self._send_position_command({
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        # トルク命令側にのみJoint1を流す
        self._send_effort_command({'joint1': tau_fric})
        
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        angle_error = abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1']))
        if angle_error <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 1 reached target angle. Moving to Step 3.")
            self._send_position_command({'joint1': self.target_q['joint1']})
            self._send_internal_moment_command(0, 0.0)
            self.current_step = SequenceStep.JOINT3_PRETENSION
            self.step_start_time = rospy.Time.now()
    
    def _step_joint3_pretension(self):
        """Step 3: Joint 3 の予張力生成"""
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        if elapsed >= STEP_DURATIONS[SequenceStep.JOINT3_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 3 (JOINT3_PRETENSION) completed. Moving to Step 4.")
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint3_deform(self):
        """Step 4: Joint 3 の空力変形"""
        if self.control_mode['joint3'] != ControlState.UNLOCKED:
            self._set_control_mode('joint3', ControlState.UNLOCKED)
        
        tau_fric = self._calculate_friction_torque('joint3')
        
        # 修正：位置命令にはJoint3を含めない
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2']
        })
        self._send_effort_command({'joint3': tau_fric})
        
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        angle_error = abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3']))
        if angle_error <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Joint 3 reached target angle. Moving to Step 5.")
            self._send_position_command({'joint3': self.target_q['joint3']})
            self._send_internal_moment_command(2, 0.0)
            self.joint2_current_target = self.current_q['joint2']
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()
    
    def _step_joint2_servo(self):
        """Step 5: Joint 2 のサーボ変形"""
        angle_diff = self._get_angle_difference(self.joint2_current_target, self.target_q['joint2'])
        if abs(angle_diff) > JOINT2_RAMP_RATE:
            if angle_diff > 0