#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（協調空力駆動・完全修正版）

使用例:
  python hydrus_xi_deformation_sequence.py 0.0 0.4 -1.0
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum

# ======================== 定数定義 ========================

class SequenceStep(Enum):
    """シーケンスのステップ定義"""
    INIT = 0                      # 初期ホバリング
    JOINT1_PRETENSION = 1         # Joint 1 の予張力生成
    JOINT1_DEFORM = 2             # Joint 1 の協調空力変形
    JOINT3_PRETENSION = 3         # Joint 3 の予張力生成
    JOINT3_DEFORM = 4             # Joint 3 の協調空力変形
    JOINT2_SERVO = 5              # Joint 2 のサーボ変形
    COMPLETE = 6                  # 完了

# パラメータ（調整可能）
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT_RAMP_RATE = 0.01           # 全関節共通のスロープ速度 [rad/loop]

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
        
        # ===== 各関節のスロープ追従用ターゲットターゲット =====
        self.joint1_current_target = 0.0
        self.joint2_current_target = 0.0
        self.joint3_current_target = 0.0
        
        # ===== シーケンス状態 =====
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        
        # ===== ROS パブリッシャ =====
        self.joints_ctrl_pub = rospy.Publisher(
            '/hydrus_xi/joints_ctrl',
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
    
    def _send_position_command(self, joints_dict):
        """位置指令を送信（すべてのステップで3関節の協調維持のために常時送信）"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for joint_name in ['joint1', 'joint2', 'joint3']:
            if joint_name in joints_dict:
                msg.name.append(joint_name)
                msg.position.append(joints_dict[joint_name])
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        self.joints_ctrl_pub.publish(msg)

    def _send_internal_moment_command(self, joint_idx, tau_des):
        """C++のフライトコントローラに内部モーメント補償を送信"""
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)
    
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
            # 次の変形ステップに向けて、現在の角度をスロープの開始地点として登録
            self.joint1_current_target = self.current_q['joint1']
            self.current_step = SequenceStep.JOINT1_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint1_deform(self):
        """Step 2: Joint 1 の協調空力変形（★スロープ指令+モーメント同期）"""
        angle_diff = self._get_angle_difference(self.joint1_current_target, self.target_q['joint1'])
        if abs(angle_diff) > JOINT_RAMP_RATE:
            if angle_diff > 0: self.joint1_current_target += JOINT_RAMP_RATE
            else: self.joint1_current_target -= JOINT_RAMP_RATE
        else:
            self.joint1_current_target = self.target_q['joint1']
        
        # 3関節すべてに正しい位置命令を送り、形状変化をシミュレータに追従させる
        self._send_position_command({
            'joint1': self.joint1_current_target,
            'joint2': self.current_q['joint2'],
            'joint3': self.current_q['joint3']
        })
        
        # フライトコントローラへの空力補償モーメントを完全に同期送信
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        angle_error = abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1']))
        if angle_error <= ANGLE_ERROR_THRESHOLD and self.joint1_current_target == self.target_q['joint1']:
            rospy.loginfo("[HydrusXiSequencer] Joint 1 reached target angle. Moving to Step 3.")
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
            # 次の変形ステップに向けて開始地点を登録
            self.joint3_current_target = self.current_q['joint3']
            self.current_step = SequenceStep.JOINT3_DEFORM
            self.step_start_time = rospy.Time.now()
    
    def _step_joint3_deform(self):
        """Step 4: Joint 3 の協調空力変形（★スロープ指令+モーメント同期）"""
        angle_diff = self._get_angle_difference(self.joint3_current_target, self.target_q['joint3'])
        if abs(angle_diff) > JOINT_RAMP_RATE:
            if angle_diff > 0: self.joint3_current_target += JOINT_RAMP_RATE
            else: self.joint3_current_target -= JOINT_RAMP_RATE
        else:
            self.joint3_current_target = self.target_q['joint3']
        
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.current_q['joint2'],
            'joint3': self.joint3_current_target
        })
        
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        angle_error = abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3']))
        if angle_error <= ANGLE_ERROR_THRESHOLD and self.joint3_current_target == self.target_q['joint3']:
            rospy.loginfo("[HydrusXiSequencer] Joint 3 reached target angle. Moving to Step 5.")
            self._send_internal_moment_command(2, 0.0)
            self.joint2_current_target = self.current_q['joint2']
            self.current_step = SequenceStep.JOINT2_SERVO
            self.step_start_time = rospy.Time.now()
    
    def _step_joint2_servo(self):
        """Step 5: Joint 2 のサーボ変形"""
        angle_diff = self._get_angle_difference(self.joint2_current_target, self.target_q['joint2'])
        if abs(angle_diff) > JOINT_RAMP_RATE:
            if angle_diff > 0: self.joint2_current_target += JOINT_RAMP_RATE
            else: self.joint2_current_target -= JOINT_RAMP_RATE
        else:
            self.joint2_current_target = self.target_q['joint2']
        
        self._send_position_command({
            'joint1': self.current_q['joint1'],
            'joint2': self.joint2_current_target,
            'joint3': self.current_q['joint3']
        })
        
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            rospy.loginfo("[HydrusXiSequencer] Step 5 (JOINT2_SERVO) completed. Moving to Step 6 (COMPLETE).")
            self.current_step = SequenceStep.COMPLETE
            self.step_start_time = rospy.Time.now()
    
    def _step_complete(self):
        """Step 6: 完了状態"""
        self._send_position_command({
            'joint1': self.target_q['joint1'],
            'joint2': self.target_q['joint2'],
            'joint3': self.target_q['joint3']
        })
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 変形完了状態にあります。次のターゲット角度の入力を待機中...")
    
    # ======================== メインループ ========================
    
    def _control_loop(self, event):
        try:
            if self.current_step == SequenceStep.INIT: self._step_init()
            elif self.current_step == SequenceStep.JOINT1_PRETENSION: self._step_joint1_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT3_PRETENSION: self._step_joint3_pretension()
            elif self.current_step == SequenceStep.JOINT3_DEFORM: self._step_joint3_deform()
            elif self.current_step == SequenceStep.JOINT2_SERVO: self._step_joint2_servo()
            elif self.current_step == SequenceStep.COMPLETE: self._step_complete()
        except Exception as e:
            rospy.logerr("[HydrusXiSequencer] Error in control loop: %s", str(e))
    
    def shutdown(self):
        rospy.loginfo("[HydrusXiSequencer] Shutting down...")
        self.loop_timer.shutdown()
        rospy.loginfo("[HydrusXiSequencer] Shutdown complete.")

# ======================== メイン関数 ========================

def main():
    """メイン関数"""
    rospy.init_node('hydrus_xi_deformation_sequencer', log_level=rospy.INFO)
    
    target_q1, target_q2, target_q3 = 0.0, 0.0, 0.0
    if len(sys.argv) >= 4:
        try:
            target_q1 = float(sys.argv[1])
            target_q2 = float(sys.argv[2])
            target_q3 = float(sys.argv[3])
        except ValueError:
            rospy.logerr("[HydrusXiSequencer] Failed to parse initial arguments.")
    
    sequencer = HydrusXiDeformationSequencer(target_q1, target_q2, target_q3)
    rospy.loginfo("[HydrusXiSequencer] 連続変形シーケンス制御スクリプトが常駐を開始しました。")
    
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        if sequencer.current_step == SequenceStep.COMPLETE:
            print("\n" + "=" * 60)
            print(" 🤖 【Hydrus-Xi 連続変形コントロールシステム】")
            print(" 現在の目標角度への変形がすべて完了しました（位置保持中）。")
            print(" 次の目標関節角度 [q1 q2 q3] をスペース区切りで入力してください。")
            print("=" * 60)
            
            try:
                user_input = input("✨ 次の目標角度を入力 -> : ")
                if user_input.strip().lower() == 'q': break
                
                angles = [float(x) for x in user_input.split()]
                if len(angles) != 3:
                    print("⚠️ [入力エラー] 3つの数値をスペースで区切ってください。")
                    continue
                
                sequencer.update_target_angles(angles[0], angles[1], angles[2])
                
            except ValueError:
                print("⚠️ [入力エラー] 有効な数値を入力してください。")
            except KeyboardInterrupt:
                break
        else:
            rate.sleep()
            
    sequencer.shutdown()

if __name__ == '__main__':
    main()