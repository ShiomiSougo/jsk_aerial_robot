#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 連続変形シーケンス実行スクリプト（サラサラURDF・安全ソフトランディング版）
変形順序: Joint 1 (空力) ➔ Joint 2 & 3 (サーボ同時変形)

【修正1・最重要】センチネル値 target_joint_index=-1 でモーメント制御OFF
【修正4】single message for Joint2&3 Servo to avoid race condition
【修正5】list_controllers で実状態を検証してから状態遷移する
【修正6】Joint 1 の空力変形トルク計算を「1->2->3 版」の実装に差し替え
        P_GAIN 0.03->0.20 / MAX_DRIVE_TORQUE_BASE 0.25->0.18 / MIN_FRICTION_TORQUE 0.18->0.12

【修正7】Joint 1 プリロードの符号バグ修正 + 解放時トルク段差の解消

  ■ 背景（C++ 側 hydrus_xi_under_actuated_navigation.cpp より）
    computeExactInternalMoment() は theta[i] = theta[i-1] + q[i-1] で運動学を組み、
    (r × F) の z 成分を返す。したがって符号規約は

        tau_internal > 0  ⟺  q が増加する方向

    よって DEFORM 側の tau_des = P_GAIN * (target - current) は向きとして正しい。

  ■ 旧実装の不具合
    プリロードは current_preload = PRELOAD_TORQUE(+0.40) * progress で
    「常に +0.40 に向かって単調増加」しており、目標角の符号を見ていなかった。
    q1 を減らす変形（ログのラン1: 1.5675->0.9、ラン2: 0.7303->0.5）では
    プリロードが変形と真逆、しかも大きさは MAX_DRIVE_TORQUE_BASE(0.18) の 2.2 倍。

    さらに tau_des_target_ は COBYLA の目的関数に soft penalty
    (w_tau = 2000.0, 二乗誤差) として乗るだけで、ジンバル配置の組み替えを伴う。
    gimbal_delta_angle_ = 0.5 rad/周期 の制限により、+0.40 -> -0.18 の反転には
    実測で約 0.45 秒（plan 20Hz で 9 周期）を要していた。
    その 0.45 秒がまるごとサーボ STOP 後に来るため、自由になった関節が
    目標と逆向きに押され続けていた（オーバーシュート 0.17〜0.22 rad の主因）。

  ■ 今回の修正
    (a) プリロードのトルクを毎ループ _calculate_target_moment('joint1') から取得し、
        符号を変形方向に一致させる。
    (b) ランプ終端値を DEFORM の初期トルクと同一にすることで、
        サーボ STOP 時のトルク段差をゼロにする（ジンバル反転待ちが不要になる）。
    (c) |angle_diff| <= ANGLE_ERROR_THRESHOLD なら PRETENSION / DEFORM を丸ごとスキップ。
        （旧実装ではラン3のように、動かす必要が無くても +0.40 N・m を 2 秒印加していた）

  ■ 未対応（別途指示があれば対応）
    - DEFORM 終了条件に速度判定が無く、離脱速度ぶんの惰行が残る
    - DECEL_ZONE == ANGLE_ERROR_THRESHOLD == 0.05 のため減速帯が実質1ループで抜ける
    - プリロード中に joint_targets を毎ループ current_q で上書きするため、
      サーボ偏差が消えて関節がクリープする
    - C++ 側の target_joint_index_ / tau_des_target_ / has_moment_command_ が
      spinner スレッドと plan_thread_ 間で無保護（COBYLA 評価中に切り替わり得る）
    - C++ 側でジンバル角が正規化されず run をまたいで単調ドリフトする

使用例:
  python hydrus_xi_deformation_sequence.py -0.3 1.0 -0.3
"""

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import (
    SwitchController, SwitchControllerRequest,
    ListControllers, ListControllersRequest,
)
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

# ★ 【修正7】プリロード値は _calculate_target_moment() の戻り値へのランプに置き換えたため未使用。
#            互換のため定数のみ残置する。
PRELOAD_TORQUE = 0.40  # DEPRECATED: 現在どこからも参照されない

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

        # ★ 【修正5】実状態検証用に list_controllers サービスを取得
        rospy.wait_for_service('/hydrus_xi/controller_manager/list_controllers')
        self.list_ctrl_client = rospy.ServiceProxy('/hydrus_xi/controller_manager/list_controllers', ListControllers)
        
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

        # ★ 【修正5】起動時点で joint1 サーボが running であることを確定させておく
        self._ensure_controller_state('joint1', want_running=True)

        self._send_synchronized_command()
        # ★ 【修正1・最重要】初期化時点でセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        rospy.loginfo("[HydrusXiSequencer] 🟩 コントローラ接続確立。初期姿勢+モーメント制御OFF送信。")

        rospy.loginfo("[HydrusXiSequencer] Initialized: q1_target=%.3f, q2_target=%.3f, q3_target=%.3f (Aerodynamic -> Servo Mode)", 
                     target_q1, target_q2, target_q3)
        self.loop_timer = rospy.Timer(rospy.Duration(DT), self._control_loop)
        
    def _switch_joint_controller(self, joint_key, action):
        """ROS Control の動的スイッチ（サービス呼び出し本体）"""
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

    # ======================== ★ 【修正5】実状態検証つきスイッチ ========================

    @staticmethod
    def _controller_name_matches(full_name, listed_name):
        """
        switch で使うフルパス名と list_controllers が返す名前を、
        末尾のパス要素で柔軟に照合する。
        """
        a = [s for s in full_name.strip('/').split('/') if s]
        b = [s for s in listed_name.strip('/').split('/') if s]
        if not a or not b:
            return False
        n = min(len(a), len(b))
        return a[-n:] == b[-n:]

    def _get_controller_state(self, joint_key):
        """
        controller_manager から実際の稼働状態を取得する。
        戻り値: 'running' / 'stopped' / 'initialized' / None(取得不可・不一致)
        """
        controller_name = JOINT_CONTROLLERS[joint_key]
        try:
            res = self.list_ctrl_client(ListControllersRequest())
        except rospy.ServiceException as e:
            rospy.logerr("[HydrusXiSequencer] list_controllers 呼び出し失敗: %s", str(e))
            return None
        for c in res.controller:
            if self._controller_name_matches(controller_name, c.name):
                return c.state
        return None

    def _ensure_controller_state(self, joint_key, want_running, retries=10, settle=0.05):
        """
        ★ 【修正5・最重要】switch_controller の ok 応答だけを信用せず、
           list_controllers で実際の状態を確認し、目標状態になるまでリトライする。
        """
        desired = 'running' if want_running else 'stopped'
        action = 'start' if want_running else 'stop'

        for _ in range(retries):
            state = self._get_controller_state(joint_key)
            if state == desired:
                return True
            if state is None:
                # 状態確認ができない環境では従来通りスイッチ結果を信用（デッドロック回避）
                if self._switch_joint_controller(joint_key, action):
                    rospy.sleep(settle)
                    return True
                rospy.sleep(settle)
                continue
            self._switch_joint_controller(joint_key, action)
            rospy.sleep(settle)

        final_state = self._get_controller_state(joint_key)
        if final_state == desired:
            return True
        rospy.logerr("[HydrusXiSequencer] ❌ %s を %s にできませんでした（現在: %s）",
                     JOINT_CONTROLLERS[joint_key], desired, final_state)
        return False

    def update_target_angles(self, q1, q2, q3):
        """
        ★ 【安全バッファ消滅版】
        Gazeboをクラッシュさせる SwitchController を使わず、
        現在角のホールドによってPIDの残存トルク（I項）を安全にゼロクリアする。
        """
        rospy.loginfo("[HydrusXiSequencer] 🔄 新目標の処理を開始（安全バッファクリア）...")

        # 1. まず現在の実角度をターゲット値として同期バッファに完全に上書き
        for j in self.all_joint_names:
            self.joint_targets[j] = self.current_q[j]

        # 2. モーメント制御を完全に OFF
        self._send_internal_moment_command(-1, 0.0)

        # 3. コントローラは running のまま、現在角コマンドを数回パブリッシュして
        #    内部の偏差および累積積分項（I項）を安全にお掃除する
        rate = rospy.Rate(20)
        for _ in range(5):
            self._send_synchronized_command()
            rate.sleep()

        # 4. 蓄積が抜けた状態で、ターゲット配列を次の目標値に更新
        self.target_q['joint1'] = q1
        self.target_q['joint2'] = q2
        self.target_q['joint3'] = q3

        # 5. ステートマシンの完全初期化
        self.current_step = SequenceStep.INIT
        self.step_start_time = rospy.Time.now()
        self.stabilize_loop_count = 0
        
        rospy.loginfo("[HydrusXiSequencer] 🟩 安全初期化完了。q1=%.3f, q2=%.3f, q3=%.3f で再始動。", q1, q2, q3)
        
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

        注意: OFF は「内部モーメントが 0 になる」ことを意味しない。
              C++ 側では computeMomentPenalty() が 0.0 を返すだけで、
              以後の内部モーメントは FCTmin 最大化の成り行きとなる。
        """
        msg = Float64MultiArray()
        msg.data = [float(joint_idx), float(tau_des)]
        self.moment_pub.publish(msg)

    def _calculate_target_moment(self, joint_name):
        """
        ★ 【修正6】空力変形用モーメント計算（1->2->3 版の実装を反映）

        符号規約（C++ computeExactInternalMoment() より）:
            tau > 0  ⟺  q が増加する方向
        したがって tau_des = P_GAIN * (target - current) で向きは正しい。

        トルク特性（新ゲイン）:
            |diff| > 0.90 rad        : ±0.18            （上限クリップ）
            0.60 < |diff| < 0.90 rad : 0.20 * diff      （比例帯 0.12〜0.18）
            0.05 < |diff| < 0.60 rad : ±0.12            （摩擦補償の床）
            |diff| < 0.05 rad        : 0.20 * diff      （減速帯・摩擦補償なし）
        """
        angle_diff_to_final = self._get_angle_difference(self.current_q[joint_name], self.target_q[joint_name])

        P_GAIN = 0.12
        MAX_DRIVE_TORQUE_BASE = 0.18
        MIN_FRICTION_TORQUE = 0.12

        tau_des = P_GAIN * angle_diff_to_final
        remaining_angle = abs(angle_diff_to_final)

        # 摩擦補償：誤差が閾値以上あるのにトルクが小さすぎる場合は底上げする
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
        """
        ★ 【修正7】Joint1 のプリロード段階

        旧: current_preload = PRELOAD_TORQUE(+0.40) * progress   ← 符号固定・変形と逆向き
        新: tau_deform（符号付き・毎ループ再計算）へのランプ

        これにより
          - プリロードの向きが変形方向と一致する
          - ランプ終端値が DEFORM の初期トルクと完全に一致し、
            サーボ STOP 時のトルク段差がゼロになる
            （＝ジンバル配置の符号反転待ち 約0.45秒 が発生しない）
        """
        self.joint_targets['joint1'] = self.current_q['joint1']
        self._send_synchronized_command()

        angle_diff = self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])

        # ★ 【修正7-c】既に目標到達 → プリロードも変形も不要
        #    （旧実装では動かす必要が無くても +0.40 N・m を 2 秒印加していた）
        if abs(angle_diff) <= ANGLE_ERROR_THRESHOLD:
            self._send_internal_moment_command(-1, 0.0)
            rospy.loginfo("[HydrusXiSequencer] Joint 1 は既に目標角（diff=%.4f） ➔ 変形をスキップして Step 3 へ",
                          angle_diff)
            self.stabilize_loop_count = 0
            self.current_step = SequenceStep.JOINT1_STABILIZE
            self.step_start_time = rospy.Time.now()
            return

        elapsed = (rospy.Time.now() - self.step_start_time).to_sec()
        duration = STEP_DURATIONS[SequenceStep.JOINT1_PRETENSION]
        progress = min(1.0, elapsed / duration)

        # ★ 【修正7-a,b】終端で DEFORM の初期トルクに一致させる（符号も大きさも）
        tau_deform = self._calculate_target_moment('joint1')
        tau_cmd = tau_deform * progress
        self._send_internal_moment_command(0, tau_cmd)

        rospy.loginfo_throttle(0.5, "[Joint1Preload] progress=%.2f, tau_cmd=%.4f (target=%.4f), angle_diff=%.4f",
                               progress, tau_cmd, tau_deform, angle_diff)

        # ★ トルクが立ち上がりきってから解放する（ジンバル移動待ちを含む）
        if elapsed >= duration:
            if self._ensure_controller_state('joint1', want_running=False):
                rospy.loginfo("[HydrusXiSequencer] Step 1 完了（tau=%.4f で解放） ➔ Step 2 (Joint 1 純空力変形開始)",
                              tau_cmd)
                self.current_step = SequenceStep.JOINT1_DEFORM
                self.step_start_time = rospy.Time.now()

    def _step_joint1_deform(self):
        """Joint1 を空力モーメントで変形（サーボ OFF）"""
        self.joint_targets['joint1'] = self.current_q['joint1']
        self._send_synchronized_command()
        
        # ★ 【修正1】符号付きモーメント指令を毎ステップ送信（target_joint_index=0）
        tau_des = self._calculate_target_moment('joint1')
        self._send_internal_moment_command(0, tau_des)
        
        if abs(self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1'])) <= ANGLE_ERROR_THRESHOLD:
            # ★ 【修正1】モーメント制御OFF（センチネル値）
            self._send_internal_moment_command(-1, 0.0)
            self.joint_targets['joint1'] = self.current_q['joint1']
            self._send_synchronized_command()
            
            # ★ 【修正5】start が「実際に running になったか」を検証してから遷移
            if self._ensure_controller_state('joint1', want_running=True):
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
                rospy.loginfo("[HydrusXiSequencer] Joint 1 静定完了（q1=%.4f, 目標=%.4f, 誤差=%.4f） ➔ Step 4 (Joint 2 & 3 同時サーボ開始)",
                              self.current_q['joint1'], self.target_q['joint1'],
                              self._get_angle_difference(self.current_q['joint1'], self.target_q['joint1']))
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
        # ★ 【修正1・最重要】STABILIZE フェーズではセンチネル値（-1）でモーメント制御OFF
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
        # ★ 【修正1・最重要】COMPLETE フェーズではセンチネル値（-1）でモーメント制御OFF
        self._send_internal_moment_command(-1, 0.0)
        if (rospy.Time.now() - self.step_start_time).to_sec() < 0.1:
            rospy.loginfo("[HydrusXiSequencer] 🎉 全変形シーケンス完走！ (q1=%.4f/%.4f, q2=%.4f/%.4f, q3=%.4f/%.4f)",
                          self.current_q['joint1'], self.target_q['joint1'],
                          self.current_q['joint2'], self.target_q['joint2'],
                          self.current_q['joint3'], self.target_q['joint3'])

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