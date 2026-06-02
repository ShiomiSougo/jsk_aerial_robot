#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 空力推力駆動変形シーケンス実行スクリプト

使用例:
  python hydrus_xi_deformation_sequence.py 0.5 0.3 -0.5
  
Target angles:
  target_q1: Joint 1 の目標角度 [rad]
  target_q2: Joint 2 の目標角度 [rad]
  target_q3: Joint 3 の目標角度 [rad]
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
FRICTION_COEFF = 0.1             # 摩擦係数 [N*m*s/rad]
FRICTION_MARGIN = 1.05           # 摩擦マージン（105%）
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
        """
        初期化
        
        Args:
            target_q1 (float): Joint 1 の目標角度 [rad]
            target_q2 (float): Joint 2 の目標角度 [rad]
            target_q3 (float): Joint 3 の目標角度 [rad]
        """
        
        # ===== ターゲット角度 =====
        self.target_q = {
            'joint1': target_q1,
            'joint2': target_q2,
            'joint3': target_q3
        }
        
        # ===== 現在の関節状態 =====
        self.current_q = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0
        }
        self.current_dq = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0
        }
        
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
        
        # ===== ROS パブリッシャ =====
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
        self.loop_timer = rospy.Timer(
            rospy.Duration(DT),
            self._control_loop
        )
    
    # ======================== コールバック関数 ========================
    
    def _joint_state_callback(self, msg):
        """
        /hydrus_xi/joint_states トピックのコールバック
        
        Args:
            msg (sensor_msgs/JointState): 関節状態メッセージ
        """
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
        """角度を [-pi, pi] の範囲に正規化"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _get_angle_difference(self, current, target):
        """現在角度と目標角度の差分を計算（最短経路）"""
        diff = target - current
        return self._normalize_angle(diff)
    
    def _set_control_mode(self, joint_name, mode):
        """指定関節の制御モードを変更"""
        self.control_mode[joint_name] = mode
        rospy.loginfo(
            "[HydrusXiSequencer] %s control mode: %s",
            joint_name, mode.name
        )
    
    def _send_position_command(self, joints_dict):
        """
        位置指令を送信（Lockモード）
        
        Args:
            joints_dict (dict): {'joint1': pos1, 'joint2': pos2, 'joint3': pos3}
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        
        for joint_name in ['joint1', 'joint2', 'joint3']:
            if joint_name in joints_dict:
                msg.name.append(joint_name)
                msg.position.append(joints_dict[joint_name])
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        
        self.joints_ctrl_pub.publish(msg)
    
    def _send_effort_command(self, joints_dict):
        """
        エフォート指令を送信（Unlockモード、摩擦トルク）
        
        Args:
            joints_dict (dict): {'joint1': torque1, 'joint2': torque2, 'joint3': torque3}
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        
        for joint_name in ['joint1', 'joint2', 'joint3']:
            if joint_name in joints_dict:
                msg.name.append(joint_name)
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(joints_dict[joint_name])
        
        self.joints_torque_ctrl_pub.publish(msg)
    
    def _send_internal_moment_command(self, joint_idx, tau_des):
        """
        C++ノードへ内部モーメント指令を送信
        
        Args:
            joint_idx (int): 対象関節インデックス (0=joint1, 1=joint2, 2=joint3)
            tau_des (float): 目標内部モーメント [N*m]
        """
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)
        
        rospy.logdebug(
            "[HydrusXiSequencer] Sent internal moment: joint_idx=%d, tau_des=%.4f",
            joint_idx, tau_des
        )
    
    def _calculate_friction_torque(self, joint_name):
        """摩擦トルクを計算（符号付き）"""
        dq = self.current_dq[joint_name]
        tau_fric = -FRICTION_COEFF * dq
        return tau_fric
    
    def _calculate_pretension_moment(self, joint_name, joint_idx):
        """予張力モーメントを計算（最短経路を考慮した符号付き）"""
        angle_diff = self._get_angle_difference(
            self.current_q[joint_name],
            self.target_q[joint_name]
        )
        
        friction_mag = abs(self._calculate_friction_torque(joint_name))
        tau_pretension = FRICTION_MARGIN * friction_mag
        
        if angle_diff >= 0:
            tau_pretension = abs(tau_pretension)
        else:
            tau_pretension = -abs(tau_pretension)
        
        return tau_pretension
    
    # ======================== ステップ実行関数 ========================
    
    def _step_init(self):
        """Step 0: 初期ホバリング"""
        # 全関節を現在値で位置制御
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        # 内部モーメントをリセット
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if elapsed >= STEP_DURATIONS[SequenceStep.INIT]:
            rospy.loginfo("[HydrusXiSequencer] Step 0 (INIT) completed. Moving to Step 1.")
            self.current_step = SequenceStep.JOINT1_PRETENSION
            self.step_start_time = rospy.Time.now()
    
    def _step_joint1_pretension(self):
        """Step 1: Joint 1 の予張力生成"""
        # 全関節を現在値で位置制御（Joint 1 はLock）
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        # Joint 1 の予張力モーメントを計算・送信
        tau_des = self._calculate_pretension_moment('joint1', 0)
        self._send_internal_moment_command(0, tau_des)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if elapsed >= STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 1 (JOINT1_PRETENSION) completed. Moving to Step 2.")
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint1_deform(self):
        """Step 2: Joint 1 の空力変形"""
        # Joint 1 をUnlockモードに切り替え
        if self.control_mode['joint1'] != ControlState.UNLOCKED:
            self._set_control_mode('joint1', ControlState.UNLOCKED)
        
        # Joint 1 の摩擦トルクを計算・送信
        tau_fric = self._calculate_friction_torque('joint1')
        
        # Joint 2, 3 は位置制御で保持、Joint 1 はトルク制御
        self._send_position_command({
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        self._send_effort_command({
            'joint1': tau_fric
        })
        
        # 目標角度との誤差をチェック
        angle_error = abs(self._get_angle_difference(
            self.current_q['joint1'],
            self.target_q['joint1']
        ))
        
        if angle_error <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo(
                "[HydrusXiSequencer] Joint 1 reached target angle (error=%.4f). "
                "Moving to Step 3.",
                angle_error
            )
            # Joint 1 をLockモードに戻す
            self._send_position_command({'joint1': self.target_q['joint1']})
            self._send_internal_moment_command(0, 0.0)
            
            self.current_step = SequenceStep.JOINT3_PRETENSION
            self.step_start_time = rospy.Time.now()
    
    def _step_joint3_pretension(self):
        """Step 3: Joint 3 の予張力生成"""
        # 全関節を現在値で位置制御（Joint 3 はLock）
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        # Joint 3 の予張力モーメントを計算・送信
        tau_des = self._calculate_pretension_moment('joint3', 2)
        self._send_internal_moment_command(2, tau_des)
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if elapsed >= STEP_DURATIONS[SequenceStep.JOINT3_PRETENSION]:
            rospy.loginfo("[HydrusXiSequencer] Step 3 (JOINT3_PRETENSION) completed. Moving to Step 4.")
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint3_deform(self):
        """Step 4: Joint 3 の空力変形"""
        # Joint 3 をUnlockモードに切り替え
        if self.control_mode['joint3'] != ControlState.UNLOCKED:
            self._set_control_mode('joint3', ControlState.UNLOCKED)
        
        # Joint 3 の摩擦トルクを計算・送信
        tau_fric = self._calculate_friction_torque('joint3')
        
        # Joint 1, 2 は位置制御で保持、Joint 3 はトルク制御
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2']
        })
        
        self._send_effort_command({
            'joint3': tau_fric
        })
        
        # 目標角度との誤差をチェック
        angle_error = abs(self._get_angle_difference(
            self.current_q['joint3'],
            self.target_q['joint3']
        ))
        
        if angle_error <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo(
                "[HydrusXiSequencer] Joint 3 reached target angle (error=%.4f). "
                "Moving to Step 5.",
                angle_error
            )
            # Joint 3 をLockモードに戻す
            self._send_position_command({'joint3': self.target_q['joint3']})
            self._send_internal_moment_command(2, 0.0)
            
            # Joint 2 スロープ制御の初期化
            self.joint2_current_target = self.current_q['joint2']
            
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()
    
    def _step_joint2_servo(self):
        """Step 5: Joint 2 のサーボ変形"""
        # Joint 1, 3 は現在値で保持
        # 内部モーメント指令はリセット
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint3': self.current_q['joint3']
        })
        
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        # Joint 2 の目標値をスロープ更新
        angle_diff = self._get_angle_difference(
            self.joint2_current_target,
            self.target_q['joint2']
        )
        
        if abs(angle_diff) > JOINT2_RAMP_RATE:
            # まだ目標に到達していない → スロープ更新
            if angle_diff > 0:
                self.joint2_current_target += JOINT2_RAMP_RATE
            else:
                self.joint2_current_target -= JOINT2_RAMP_RATE
        else:
            # 目標に到達した
            self.joint2_current_target = self.target_q['joint2']
        
        # Joint 2 に位置指令を送信
        self._send_position_command({'joint2': self.joint2_current_target})
        
        # 目標到達確認
        if abs(self._get_angle_difference(
            self.current_q['joint2'],
            self.target_q['joint2']
        )) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Step 5 (JOINT2_SERVO) completed. Moving to Step 6 (COMPLETE).")
            self.current_step = SequenceStep.COMPLETE
            self.step_start_time = rospy.Time.now()
    
    def _step_complete(self):
        """Step 6: 完了"""
        self._send_position_command({
            'joint1': self.target_q['joint1'],
            'joint2': self.target_q['joint2'],
            'joint3': self.target_q['joint3']
        })
        
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        rospy.loginfo(
            "[HydrusXiSequencer] Deformation sequence COMPLETE. "
            "Final angles: q1=%.3f, q2=%.3f, q3=%.3f [rad]",
            self.current_q['joint1'], self.current_q['joint2'], self.current_q['joint3']
        )
    
    # ======================== メインループ ========================
    
    def _control_loop(self, event):
        """メインコントロールループ（20 Hz）"""
        try:
            if self.current_step == SequenceStep.INIT:
                self._step_init()
            elif self.current_step == SequenceStep.JOINT1_PRETENSION:
                self._step_joint1_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM:
                self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT3_PRETENSION:
                self._step_joint3_pretension()
            elif self.current_step == SequenceStep.JOINT3_DEFORM:
                self._step_joint3_deform()
            elif self.current_step == SequenceStep.JOINT2_SERVO:
                self._step_joint2_servo()
            elif self.current_step == SequenceStep.COMPLETE:
                self._step_complete()
        
        except Exception as e:
            rospy.logerr(
                "[HydrusXiSequencer] Error in control loop: %s",
                str(e)
            )
    
    def shutdown(self):
        """シャットダウン処理"""
        rospy.loginfo("[HydrusXiSequencer] Shutting down...")
        self.loop_timer.shutdown()
        rospy.loginfo("[HydrusXiSequencer] Shutdown complete.")


# ======================== メイン関数 ========================

def main():
    """メイン関数"""
    rospy.init_node('hydrus_xi_deformation_sequencer', log_level=rospy.INFO)
    
    if len(sys.argv) < 4:
        rospy.logerr(
            "[HydrusXiSequencer] Usage: python hydrus_xi_deformation_sequence.py "
            "<target_q1> <target_q2> <target_q3>"
        )
        rospy.logerr("  Example: python hydrus_xi_deformation_sequence.py 0.5 0.3 -0.5")
        sys.exit(1)
    
    try:
        target_q1 = float(sys.argv[1])
        target_q2 = float(sys.argv[2])
        target_q3 = float(sys.argv[3])
    except ValueError:
        rospy.logerr("[HydrusXiSequencer] Failed to parse target angles as float.")
        sys.exit(1)
    
    sequencer = HydrusXiDeformationSequencer(target_q1, target_q2, target_q3)
    
    rospy.loginfo("[HydrusXiSequencer] Deformation sequence started.")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[HydrusXiSequencer] Interrupted by user.")
    finally:
        sequencer.shutdown()


if __name__ == '__main__':
    main()