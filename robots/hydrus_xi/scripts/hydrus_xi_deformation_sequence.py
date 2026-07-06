#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（サラサラURDF・安全ソフトランディング版）
変形順序変更版: Joint 1 (空力) ➔ Joint 2 & 3 (サーボ同時変形)

使用例:
  python hydrus_xi_deformation_sequence.py -0.3 1.0 -0.3
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from enum import Enum

class SequenceStep(Enum):
    INIT = 0                      # 初期ホバリング
    JOINT1_PRETENSION = 1         # Joint 1 のプリロード
    JOINT1_DEFORM = 2             # ① Joint 1 の純空力変形（コントローラ停止フェーズ）
    JOINT1_STABILIZE = 3          # └ Joint 1 変形後の機体揺れ収束待ち（コントローラ再開）
    JOINT2_3_SERVO = 4            # ② Joint 2 と Joint 3 の同時サーボ変形
    JOINT2_3_STABILIZE = 5        # └ Joint 2 と Joint 3 の変形後の静定待ち
    COMPLETE = 6                  # 完了

# パラメータ
ANGLE_ERROR_THRESHOLD = 0.05     # 角度誤差閾値 [rad]
JOINT_RAMP_RATE_BASE = 0.005     # 基本スロープ速度 [rad/loop]

# 静定判定用のパラメータ
STABILIZE_VELOCITY_THRESH = 0.01  # 静定角速度閾値 [rad/s]
STABILIZE_REQUIRED_LOOPS = 10     # 収束ループ数
STABILIZE_TIMEOUT = 4.0           # タイムアウト時間 [s]

# 物理的な予張力パラメータ
PRELOAD_TORQUE = 0.40             # Joint 1用

# ★検証用パラメータ
RELEASE_PROBE_DURATION = 0.5      # リリース後に dq をサンプリングする時間 [s]
BREAKAWAY_DQ_THRESH = 0.02        # 「有意な初速」とみなす角速度閾値 [rad/s]

# コントローラ名マッピング
JOINT_CONTROLLERS = {
    'joint1': "/hydrus_xi/servo_controller/joints/controller1/simulation",
    'joint2': "/hydrus_xi/servo_controller/joints/controller2/simulation",
    'joint3': "/hydrus_xi/servo_controller/joints/controller3/simulation"
}

STEP_DURATIONS = {
    SequenceStep.JOINT1_PRETENSION: 2.0,
}

LOOP_FREQ = 20.0                 # [Hz]
DT = 1.0 / LOOP_FREQ


def _sign_str(x, eps=1e-9):
    """符号を +/-/0 の文字列で返す（ログ可読性用）"""
    if x > eps:
        return '+'
    elif x < -eps:
        return '-'
    return '0'


class HydrusXiDeformationSequencer:
    def __init__(self, target_q1, target_q2, target_q3):
        self.target_q = {'joint1': target_q1, 'joint2': target_q2, 'joint3': target_q3}
        
        self.all_joint_names = ['joint1', 'joint2', 'joint3']
        
        # 受信トピック構造に合わせ、内部状態の辞書は全関節で保持
        full_joints = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4', 'joint1', 'joint2', 'joint3']
        self.current_q = {name: 0.0 for name in full_joints}
        self.current_dq = {name: 0.0 for name in full_joints}
        
        self.joint_targets = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        
        self.current_step = SequenceStep.INIT
        self.step_start_time = None
        self.stabilize_loop_count = 0

        # ★検証用の状態
        self.run_index = 0                 # 何回目のシーケンスか（1回目, 2回目, ...）
        self._probe_active = False         # リリース直後の dq 観測中フラグ
        self._probe_start_time = None
        self._probe_start_q1 = 0.0
        self._probe_peak_dq = 0.0          # 観測窓中に見た最大|dq|（符号付き）
        self._probe_first_dq_sign = '0'    # 最初に閾値を超えた向き
        self._release_info = {}            # リリース時点のスナップショット
        
        # サービスクライアントの初期化
        rospy.wait_for_service('/hydrus_xi/controller_manager/switch_controller')
        self.switch_ctrl_client = rospy.ServiceProxy('/hydrus_xi/controller_manager/switch_controller', SwitchController)
        
        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.moment_pub = rospy.Publisher('/hydrus_xi/target_internal_moment', Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_callback)
        
        # 最初の関節状態メッセージが届くまで待機
        rospy.loginfo("[HydrusXiSequencer] ⏳ 最初の /hydrus_xi/joint_states トピックの受信を待っています...")
        try:
            first_msg = rospy.wait_for_message('/hydrus_xi/joint_states', JointState, timeout=5.0)
            self._joint_state_callback(first_msg)
            
            self.joint_targets['joint1'] = self.current_q['joint1']
            self.joint_targets['joint2'] = self.current_q['joint2']
            self.joint_targets['joint3'] = self.current_q['joint3']
            
            rospy.loginfo("[HydrusXiSequencer] 🟩 初期状態の受信に成功しました。")
        except rospy.ROSException:
            rospy.logwarn("[HydrusXiSequencer] ⚠️ トピックの待機がタイムアウトしました。初期値 0.0 で処理を開始します。")

        # Publisherの接続確立待ち
        rospy.loginfo("[HydrusXiSequencer] ⏳ コントローラ（シミュレータ）との接続確立を待っています...")
        rate_wait = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate_wait.sleep()
            
        self._send_synchronized_command()
        rospy.loginfo("[HydrusXiSequencer] 🟩 コントローラとの接続が確立。初期姿勢維持コマンドを送信しました。")

        rospy.loginfo("[HydrusXiSequencer] Initialized: q1=%.3f, q2=%.3f, q3=%.3f (1(Aerodynamic) -> 2&3(Servo) Mode)", target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)
        
    def _switch_joint_controller(self, joint_key, action):
        """ROS Controlのサービスを叩いて動的にPIDをON/OFFする"""
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

    def _calculate_target_moment_joint1(self):
        """Joint 1 の空力変形用モーメント計算"""
        angle_diff_to_final = self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])
        
        P_GAIN = 0.2                
        MAX_DRIVE_TORQUE_BASE = 0.18 
        MIN_FRICTION_TORQUE = 0.12  

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
            
        return tau_des

    # ★検証用: リリース直後の dq を観測して初速の向きを判定する
    def _update_release_probe(self):
        """JOINT1_DEFORM 中、リリース後 RELEASE_PROBE_DURATION 秒だけ dq を監視"""
        if not self._probe_active:
            return

        elapsed = (rospy.Time.now() - self._probe_start_time).to_sec()
        dq1 = self.current_dq['joint1']

        # 観測窓中の符号付きピーク角速度を更新
        if abs(dq1) > abs(self._probe_peak_dq):
            self._probe_peak_dq = dq1

        # 最初にブレイクアウェイ閾値を超えた向きを記録
        if self._probe_first_dq_sign == '0' and abs(dq1) >= BREAKAWAY_DQ_THRESH:
            self._probe_first_dq_sign = _sign_str(dq1)
            rospy.loginfo(
                "[PROBE] run#%d ブレイクアウェイ検出: t=%.2fs 後, dq1=%.4f (向き %s)",
                self.run_index, elapsed, dq1, self._probe_first_dq_sign
            )

        # 観測窓終了 → サマリを1行で出力
        if elapsed >= RELEASE_PROBE_DURATION:
            self._probe_active = False
            info = self._release_info
            moved = self.current_q['joint1'] - self._probe_start_q1

            # 実測初速の向きが目標方向と一致したか
            desired_sign = info.get('desired_sign', '0')
            observed_sign = self._probe_first_dq_sign if self._probe_first_dq_sign != '0' else _sign_str(self._probe_peak_dq)
            verdict = "一致(目標へ)" if (observed_sign == desired_sign and observed_sign != '0') else \
                      "逆行(目標と反対)" if observed_sign != '0' else "初速ほぼ無し"

            rospy.logwarn(
                "[PROBE-SUMMARY] ============================================\n"
                "  run#%d (%d回目のシーケンス)\n"
                "  リリース時: q1=%.3f, target_q1=%.3f, angle_diff=%.3f\n"
                "  幾何コンテキスト: q2=%.3f, q3=%.3f (機体形状)\n"
                "  【符号比較】 目標方向=%s / プリロード指令=%s / 実測初速=%s\n"
                "  ピーク角速度 dq1=%.4f rad/s, 0.5s間の移動量=%.4f rad\n"
                "  判定: 解放直後 joint1 は %s\n"
                "============================================",
                self.run_index, self.run_index,
                info.get('q1', 0.0), info.get('target_q1', 0.0), info.get('angle_diff', 0.0),
                info.get('q2', 0.0), info.get('q3', 0.0),
                desired_sign, info.get('preload_sign', '0'), observed_sign,
                self._probe_peak_dq, moved,
                verdict
            )
    
    # ======================== 各ステップの実行関数 ========================

    def _step_init(self):
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        
        vel_sum = abs(self.current_dq['joint1']) + abs(self.current_dq['joint2']) + abs(self.current_dq['joint3'])
        
        if (rospy.Time.now() - self.step_start_time).to_sec() >= 5.0 and vel_sum < 0.005:
            rospy.loginfo("[HydrusXiSequencer] 初期静止完了 ➔ Step 1へ移行")
            self.run_index += 1  # ★このシーケンスが何回目か
            self.current_step = SequenceStep.JOINT1_PRETENSION  
            self.step_start_time = rospy.Time.now()
        elif (rospy.Time.now() - self.step_start_time).to_sec() > 10.0:
            rospy.logwarn("[HydrusXiSequencer] 初期静止タイムアウト、強行移行")
            self.run_index += 1  # ★
            self.current_step = SequenceStep.JOINT1_PRETENSION  
            self.step_start_time = rospy.Time.now()

    def _step_joint1_pretension(self):
        self.joint_targets['joint1'] = self.current_q['joint1']
        self._send_synchronized_command()
        
        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        duration = STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]
        progress = min(1.0, elapsed / duration)
        
        # ★プリロード（予張力）の方向を変形方向（angle_diff）と【逆】にする
        angle_diff = self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])
        direction = -1.0 if angle_diff >= 0 else 1.0
        
        current_preload = PRELOAD_TORQUE * progress * direction
        self._send_internal_moment_command(0, current_preload)
        
        if elapsed >= duration:
            # ★リリース時点のスナップショットを保存（検証用）
            final_angle_diff = self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])
            self._release_info = {
                'q1': self.current_q['joint1'],
                'target_q1': self.target_q['joint1'],
                'angle_diff': final_angle_diff,
                'q2': self.current_q['joint2'],
                'q3': self.current_q['joint3'],
                'desired_sign': _sign_str(final_angle_diff),   # 目標へ向かうべき向き
                'preload_sign': _sign_str(direction),           # 実際にプリロードした向き
            }

            if self._switch_joint_controller('joint1', 'stop'):
                rospy.loginfo("[HydrusXiSequencer] Step 1 Completed ➔ Step 2 (Joint 1 純空力変形開始)")
                # ★リリース直後の観測窓を開始
                self._probe_active = True
                self._probe_start_time = rospy.Time.now()
                self._probe_start_q1 = self.current_q['joint1']
                self._probe_peak_dq = 0.0
                self._probe_first_dq_sign = '0'

                self.current_step = SequenceStep.JOINT1_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        self.joint_targets['joint1'] = self.current_q['joint1'] 
        self._send_synchronized_command()

        # ★リリース直後の初速観測（変形トルクとは独立に走る）
        self._update_release_probe()
        
        tau_des = self._calculate_target_moment_joint1()
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
                rospy.loginfo("[HydrusXiSequencer] Joint 1 静定完了 ➔ Step 4 (Joint 2 & Joint 3 同時サーボ開始)")
                self.current_step = SequenceStep.JOINT2_3_SERVO
                self.step_start_time = rospy.Time.now()

    def _step_joint2_3_servo(self):
        """Joint 2 と 3 を同時にサーボ駆動する（位置指令によるランプ制御）"""
        q_sum = abs(self.current_q['joint1']) + abs(self.current_q['joint2']) + abs(self.current_q['joint3'])
        ramp_reduction_factor = max(0.2, 1.0 - 0.2 * q_sum) 
        dynamic_ramp_rate = JOINT_RAMP_RATE_BASE * ramp_reduction_factor
        
        # Joint 2 と Joint 3 の目標を同時に進める
        for joint in ['joint2', 'joint3']:
            angle_diff = self._get_angle_difference(self.joint_targets[joint], self.target_q[joint])
            if abs(angle_diff) > dynamic_ramp_rate:
                self.joint_targets[joint] += math.copysign(dynamic_ramp_rate, angle_diff)
            else:
                self.joint_targets[joint] = self.target_q[joint]
            
        self._send_synchronized_command()
        
        # サーボ干渉補償トルクの無効化（空力干渉を避けるためゼロに）
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        
        # 両方が目標に達したか判定
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
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)

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
        self._send_synchronized_command()
        self._send_internal_moment_command(0, 0.0)
        self._send_internal_moment_command(2, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 全変形シーケンス（Joint1空力 -> Joint2&3サーボ）が正常に完走しました！")

    def _control_loop(self, event):
        try:
            current_time = rospy.Time.now()
            if current_time.is_zero(): return
            if self.step_start_time is None: self.step_start_time = current_time
            
            if self.current_step == SequenceStep.INIT: self._step_init()
            elif self.current_step == SequenceStep.JOINT1_PRETENSION: self._step_joint1_pretension()
            elif self.current_step == SequenceStep.JOINT1_DEFORM: self._step_joint1_deform()
            elif self.current_step == SequenceStep.JOINT1_STABILIZE: self._step_joint1_stabilize()
            elif self.current_step == SequenceStep.JOINT2_3_SERVO: self._step_joint2_3_servo()
            elif self.current_step == SequenceStep.JOINT2_3_STABILIZE: self._step_joint2_3_stabilize()
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
            print(" ✨ 【Hydrus-Xi】ROS Control 動的スイッチ変形システム（Joint1空力 -> 2&3同時サーボ）")
            print(" 次の目標関節角度 [q1 q2 q3] を入力してください。")
            print("="*60)
            try:
                user_input = input("💡 ターゲット入力 -> : ")
                if user_input.strip().lower() == 'q': break
                angles = [float(x) for x in user_input.split()]
                if len(angles) == 3: sequencer.update_target_angles(angles[0], angles[1], angles[2])
            except (ValueError, KeyboardInterrupt, EOFError): break
        else:
            rate.sleep()
    sequencer.shutdown()

if __name__ == '__main__':
    main()