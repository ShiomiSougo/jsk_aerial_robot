#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（サラサラURDF・安全ソフトランディング版）
変形順序変更版: Joint 1 (空力) ➔ Joint 2 (サーボ) ➔ Joint 3 (空力)

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
    INIT = 0                      # 初期ホバリング
    JOINT1_3_PRETENSION = 1       # Joint 1 のプリロード（C++仕様合わせ）
    JOINT1_DEFORM = 2             # ① Joint 1 の純空力変形（コントローラ停止フェーズ）
    JOINT1_STABILIZE = 3          # └ Joint 1 変形後の機体揺れ収束待ち（コントローラ再開）
    JOINT2_SERVO = 4              # ② Joint 2 のサーボ変形
    JOINT2_STABILIZE = 5          # └ Joint 2 変形後の静定待ち（※新設：全体の対称性とバランス確保）
    JOINT3_DEFORM = 6             # ③ Joint 3 の純空力変形（コントローラ停止フェーズ）
    JOINT3_STABILIZE = 7          # └ Joint 3 変形後の機体揺れ収束待ち（コントローラ再開）
    COMPLETE = 8                  # 完了

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT_RAMP_RATE_BASE = 0.005     # 基本スロープ速度 [rad/loop]

# 静定判定用のパラメータ
STABILIZE_VELOCITY_THRESH = 0.01  # 静定角速度閾値 [rad/s]
STABILIZE_REQUIRED_LOOPS = 10     # 収束ループ数
STABILIZE_TIMEOUT = 4.0           # タイムアウト時間 [s]

# 物理的な予張力パラメータ
PRELOAD_TORQUE = 0.40             # 💡 サラサラ関節に合わせてプリロードも優しく [Nm]

# 🎯 特定された正確なコントローラ名マッピング
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
        
        # joints_ctrlトピックの送信対象はjointのみ
        self.all_joint_names = ['joint1', 'joint2', 'joint3']
        
        # 受信トピック構造（joint_states）に合わせ、内部状態の辞書は全7関節で保持
        full_joints = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4', 'joint1', 'joint2', 'joint3']
        self.current_q = {name: 0.0 for name in full_joints}
        self.current_dq = {name: 0.0 for name in full_joints}
        self.current_effort = {name: 0.0 for name in full_joints}
        
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        self.stabilize_loop_count = 0
        
        # サービスクライアントの初期化
        rospy.wait_for_service('/hydrus_xi/controller_manager/switch_controller')
        self.switch_ctrl_client = rospy.ServiceProxy('/hydrus_xi/controller_manager/switch_controller', SwitchController)
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        # 最初の関節状態メッセージが届くまで待機 ＆ 初期ターゲットの設定
        rospy.loginfo("[HydrusXiSequencer] ⏳ 最初の /hydrus_xi/joint_states トピックの受信を待っています...")
        try:
            first_msg = rospy.wait_for_message('/hydrus_xi/joint_states', JointState, timeout=5.0)
            self._joint_state_callback(first_msg)
            
            self.joint_targets['joint1'] = self.current_q['joint1']
            self.joint_targets['joint2'] = self.current_q['joint2']
            self.joint_targets['joint3'] = self.current_q['joint3']
            
            rospy.loginfo("[HydrusXiSequencer] 🟩 初期状態の受信に成功しました。(q2の初期位置: %.3f)", self.current_q['joint2'])
        except rospy.ROSException:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ トピックの待機がタイムアウトしました。初期値 0.0 で処理を開始します。")

        # Publisherの接続確立待ち
        rospy.loginfo("[HydrusXiSequencer] ⏳ コントローラ（シミュレータ）との接続確立を待っています...")
        rate_wait = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate_wait.sleep()
            
        self._send_synchronized_command()
        rospy.loginfo("[HydrusXiSequencer] 🟩 コントローラとの接続が確立。初期姿勢維持コマンドを送信しました。")

        rospy.loginfo("[HydrusXiSequencer] Initialized: q1=%.3f, q2=%.3f, q3=%.3f (1->2->3 Sequence Mode)", target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)
        
    def _switch_joint_controller(self, joint_key, action):
        """ROS Controlのサービスを叩いて動的にPIDをON/OFFするヘルパー"""
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
        angle_diff_to_final = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])
        
        P_GAIN = 0.3
        MAX_DRIVE_TORQUE_BASE = 0.08
        tau_des = P_GAIN * angle_diff_to_final
        remaining_angle = abs(angle_diff_to_final)
        
        DECEL_ZONE = 0.075 
        
        if remaining_angle < DECEL_ZONE:
            fade_factor = remaining_angle / DECEL_ZONE
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE * fade_factor
        else:
            dynamic_max_torque = MAX_DRIVE_TORQUE_BASE
            
        if tau_des > dynamic_max_torque:
            tau_des = dynamic_max_torque
        elif tau_des < -dynamic_max_torque:
            tau_des = -dynamic_max_torque
            
        return tau_des
    
    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        
        vel_sum = abs(self.current_dq['joint1']) + abs(self.current_dq['joint2']) + abs(self.current_dq['joint3'])
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= 5.0 and vel_sum < 0.005:
            rospy.loginfo("[HydrusXiSequencer] 初期静止完了 ➔ Step 1へ移行")
            self.current_step = SequenceStep.JOINT1_3_PRETENSION  
            self.step_start_time = rospy.Time.now()
        elif (rospy.Time.now() - self.step_start_time).to_sec() > 10.0:
            rospy.logwarn("[HydrusXiSequencer] 初期静止タイムアウト、強行移行")
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
                rospy.loginfo("[HydrusXiSequencer] Step 1 Completed ➔ Step 2 (Joint 1 純空力変形開始)")
                self.current_step = SequenceStep.JOINT1_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        self.joint_targets['joint1'] = self.current_q['joint1'] 
        self._send_synchronized_command()
        
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(0, 0.0)
            self.joint_targets['joint1'] = self.current_q['joint1']
            self._send_synchronized_command()
            
            if self._switch_joint_controller('joint1', 'start'):
                rospy.loginfo("[HydrusXiSequencer] Joint 1 変形完了 ➔ Step 3 (Joint 1 静定待ち)")
                self.stabilize_loop_count = 0
                self.current_step = SequenceStep.JOINT1_STABILIZE
                self.step_start_time = rospy.Time.now()

    def _step_joint1_stabilize(self):
        current_vel = abs(self.current_dq['joint1'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        if duration < 0.5:
            self.joint_targets['joint1'] = self.current_q['joint1']
            
        self._send_synchronized_command()
        
        if duration >= 0.5:
            if current_vel < STABILIZE_VELOCITY_THRESH:
                self.stabilize_loop_count += 1
            else:
                self.stabilize_loop_count = 0
                
        MIN_WAIT = 5.0
        if duration >= MIN_WAIT:
            if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
                # ⭕ 【順序変更】次は Joint 2 (位置制御) なので、ここでは controller3 ではなく何もしないで次へ
                rospy.loginfo("[HydrusXiSequencer] Joint 1 静定完了 ➔ Step 4 (Joint 2 Servo開始)")
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
        
        # サーボ干渉補償トルクの無効化
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        if abs(self._get_angle_difference(self.current_q['joint2'], self.target_q['joint2'])) <= ANGLE_ERROR_THRESHOLD:
            # ⭕ 【順序変更】Joint 2 完了後、安全のために静定待機フェーズを挟む
            rospy.loginfo("[HydrusXiSequencer] Joint 2 サーボ駆動完了 ➔ Step 5 (Joint 2 静定待ち)")
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT2_STABILIZE
            self.step_start_time = rospy.Time.now()

    def _step_joint2_stabilize(self):
        """【新設】Joint 2 変形後に機体バランスを整え、LQIのゲイン飽和を防ぐフェーズ"""
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)

        current_vel = abs(self.current_dq['joint2'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()

        if current_vel < STABILIZE_VELOCITY_THRESH:
            self.stabilize_loop_count += 1
        else:
            self.stabilize_loop_count = 0

        # しっかりと3秒姿勢を落ち着かせたあと、満を持して Joint 3 を完全フリーにする
        MIN_WAIT_J2 = 5.0
        if duration >= MIN_WAIT_J2:
            if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
                if self._switch_joint_controller('joint3', 'stop'):
                    rospy.loginfo("[HydrusXiSequencer] Joint 2 静定完了（バランス確保） ➔ Step 6 (Joint 3 純空力変形開始)")
                    self.current_step = SequenceStep.JOINT3_DEFORM
                    self.step_start_time = rospy.Time.now()

    def _step_joint3_deform(self):
        self.joint_targets['joint3'] = self.current_q['joint3']
        self._send_synchronized_command()
        
        tau_des = self._calculate_target_moment('joint3')
        self._send_internal_moment_command(2, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint3'], self.target_q['joint3'])) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(2, 0.0)
            self.joint_targets['joint3'] = self.current_q['joint3']
            self._send_synchronized_command() # ソフトランディング対応の上書きコマンド送信

            if self._switch_joint_controller('joint3', 'start'):
                rospy.loginfo("[HydrusXiSequencer] Joint 3 変形完了 ➔ Step 7 (Joint 3 静定待ち)")
                self.stabilize_loop_count = 0
                self.current_step = SequenceStep.JOINT3_STABILIZE
                self.step_start_time = rospy.Time.now()

    def _step_joint3_stabilize(self):
        current_vel = abs(self.current_dq['joint3'])
        duration = (rospy.Time.now() - self.step_start_time).to_sec()
        
        # 最初の0.5秒間は追従ホールド
        if duration < 0.5:
            self.joint_targets['joint3'] = self.current_q['joint3']

        self._send_synchronized_command()
        
        if duration >= 0.5:
            if current_vel < STABILIZE_VELOCITY_THRESH:
                self.stabilize_loop_count += 1
            else:
                self.stabilize_loop_count = 0
            
        # ⭕ 【順序変更】最後の関節なので、判定パス後はシーケンス終了(COMPLETE)へ
        MIN_WAIT_J3 = 3.0
        if duration >= MIN_WAIT_J3:
            if self.stabilize_loop_count >= STABILIZE_REQUIRED_LOOPS or duration >= STABILIZE_TIMEOUT:
                rospy.loginfo("[HydrusXiSequencer] Joint 3 静定完了 ➔ Step 8 (シーケンス完了)")
                self.current_step = SequenceStep.COMPLETE
                self.step_start_time = rospy.Time.now()

    def _step_complete(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 全空力・サーボ複合連続変形シーケンスが正常に完走しました！")

    def _control_loop(self, event):
        try:
            current_time = rospy.Time.now()
            if current_time.is_zero(): return
            if self.step_start_time is None: self.step_start_time = current_time
            
            # 💡 順序が 1 -> 2 -> 3 になるよう、条件分岐の評価順も完全連動
            if self.current_step == SequenceStep.INIT: self._step_init()
            elif self.current_step == SequenceStep.JOINT1_3_PRETENSION: self._step_joints_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT1_STABILIZE: self._step_joint1_stabilize()
            elif self.current_step == SequenceStep.JOINT2_SERVO: self._step_joint2_servo()
            elif self.current_step == SequenceStep.JOINT2_STABILIZE: self._step_joint2_stabilize()
            elif self.current_step == SequenceStep.JOINT3_DEFORM: self._step_joint3_deform()
            elif self.current_step == SequenceStep.JOINT3_STABILIZE: self._step_joint3_stabilize()
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
            print(" ✨ 【Hydrus-Xi】ROS Control 動的スイッチ変形システム（1->2->3並替版）")
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