#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）rev.22

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま joint1 の変形を成立させる。

--------------------------------------------------------------------------
rev.22 での変更（rev.21 からの差分）— controller1停止時のeffortゼロ化を強化
--------------------------------------------------------------------------
【背景】単発のeffort=0 publishでは、Gazebo側のeffortコマンドバッファが
  必ずしも上書きされないことが実験で確認されていた（研究ノート16.4節）。
  GitHub Copilotによる分析: ros_control/gazebo_ros_controlは、
  controllerをstopしてもバッファを自動でクリアしないため、1回だけの
  publishは受信タイミングによって取りこぼすことがある。

【対策】_step_joint1_try1のcontroller1停止処理を、以下のように変更した。
  - 停止の直前に、effort=0を20回、10ms間隔でpublish
  - switch_ctrlでcontroller1を停止
  - 停止の直後にも、effort=0を20回、10ms間隔でpublish
  合計40回・約0.4秒のブロッキングを伴うが、この処理はシーケンス全体で
  1回きりなので実害は小さいと判断した。

--------------------------------------------------------------------------
rev.20 での変更（rev.19 からの差分）— joint1変形前のgimbal1選択の符号修正
--------------------------------------------------------------------------
【発見】try1でa（c*pi+0.7）とb（c*pi-0.7）の両方を実際に試した結果、
  以下の対応関係が正しいことが確認された。

    目標角 - 現在角 が
      正 -> b型（c*pi - 0.7）が正しい
      負 -> a型（c*pi + 0.7）が正しい

  一方、rev.18で実装した_pick_gimbal1_target（joint1変形前のgimbal1
  固定角選択）は、この対応が逆になっていた（a=target-remainderが
  正のときa型、負のときb型を選んでいた）。

【修正】_pick_gimbal1_targetのif/elseの中身（a型/b型の対応）を入れ替え、
  try1で確認された対応関係と一致させた。_pick_try1_targetのa/b公式
  自体（a:+0.7, b:-0.7）は変更していない（これらは正しい基準として
  使われた）。

--------------------------------------------------------------------------
rev.19 での変更（rev.18 からの差分）— try1のgimbal1目標角を現在角基準に
--------------------------------------------------------------------------
【発見】rev.18のtry1（'a'=pi+0.7固定、'b'=pi-0.7固定）で、joint1変形前の
  gimbal1固定（_pick_gimbal1_target）は意図通り動いていたのに対し、
  try1側では動作が安定しない現象が確認された。

  原因: joint1変形前の固定は、常に「πに近い現在角」から近距離で
  目標角へ移動する設計になっていた（移動距離が短く安全）。一方、
  try1に入る直前のgimbal1角度は、直前のJOINT23_SERVOステップで
  gimbal1がC++側の自由最適化に委ねられているため、毎回不定である。
  そのため、固定の"pi+0.7"/"pi-0.7"を目標にすると、出発点によっては
  非常に長い距離・不定な経路の移動になり、特異形態を横切るリスクが
  生じていた。

【対策】try1の目標角を、現在のgimbal1角度を基準に選び直すよう変更した。
  a/bという物理的な向きの意味（joint1にかかる反力の向き）は変えず、
  そこへ至る経路の出発点合わせだけを行う。

    b = 現在のgimbal1角度 (self.fix_current)
    b ÷ pi の商を c、余りを d とする (b = c*pi + d)
    direction='a' -> target = c*pi + 0.7
    direction='b' -> target = c*pi - 0.7

  新規メソッド _pick_try1_target(direction) として実装し、main()の
  ASK_TRY1_DIRECTION処理から呼び出す。b, c, d, targetをすべて表示する。

  なお、joint1変形前の _pick_gimbal1_target（joint1角度ベースのルール）
  は今回変更していない。

--------------------------------------------------------------------------
rev.18 での変更（rev.17 からの差分）— 「今できていることだけ」に制限
--------------------------------------------------------------------------
問題が多岐にわたり切り分けが難しくなっていたため、gimbal1の固定角の
決め方と、joint1_try1の実行内容を、以下のとおり単純なルールに絞った。

【変更1】gimbal1の固定角の決め方を単純化。
  旧ロジック（_compute_branches / _order_branches、2分岐候補から近い方を
  選び、スルー中に特異点を踏んだら逆分岐へリトライする方式）は削除せず
  残すが、呼び出しはしない（未使用のまま保持）。

  新ロジック（_pick_gimbal1_target）:
    remainder = 現在のjoint1角度 mod pi(3.14)
    a = 目標のjoint1角度 - remainder
    a が正: gimbal1 = pi + 0.7 に固定
    a が負: gimbal1 = pi - 0.7 に固定
  この1つの候補だけを使ってスルーを開始する（_step_gimbal_fix自体の
  スルー・tau_min監視ロジックは変更していない。ただし候補が1つしか
  ないため、スルー中に特異点を踏んだ場合はリトライ先がなく、そのまま
  ABORTする＝rev.6由来の「両分枝とも踏んだ場合の中断」と同じ扱い）。

【変更2】joint2,3は rev.17 のまま（joint2 -> joint3 の逐次実行、
  JOINT23_PHASE_TIMEOUT で joint3 へ移行）。変更なし。

【変更3】joint2,3完了後、joint1_try1を実行するか選択させる。
  新しい Step.ASK_TRY1 を追加。メインループ（対話プロンプト）で
  「1」を入力すれば実行、「2」を入力すればスキップしてCOMPLETEへ。

【変更4】joint1_try1の中身を、方向選択式に変更。
  旧ロジック（gimbal1を2π分スピンさせる全周診断）はrev.17で既に
  コメントアウト済みだったが、rev.18ではこれを削除し、代わりに
  新しい Step.ASK_TRY1_DIRECTION で「変形方向は？」と質問する:
    'a' 入力: gimbal1 = pi + 0.7 に固定（このgimbal1目標値を表示）
    'b' 入力: gimbal1 = pi - 0.7 に固定（このgimbal1目標値を表示）
  選択後は rev.17 と同じ動作（gimbal1到達待ち→静定待ち→
  controller1停止→effortゼロpublish）。

--------------------------------------------------------------------------
rev.17 での変更（rev.6 からの差分）
--------------------------------------------------------------------------
【追加1】joint2,3を同時ではなく joint2 -> joint3 の順に逐次実行するよう
        変更。joint2 が JOINT23_PHASE_TIMEOUT 秒経っても目標に到達
        しなければ、joint3 の動作へ移行する。
【追加2】joint1_try1のgimbal1全周スピン診断をコメントアウトにより無効化
        （rev.18で削除、変更4に統合）。
【追加3】controller1停止直後、commandトピックへeffort=0を1回publishする
        処理を追加（ros_control仕様上バッファに残る最後の値を確認・
        是正する目的。効果は未確認だが、診断目的で残置）。

--------------------------------------------------------------------------
rev.6 での変更（rev.5 からの差分）— 残課題B（スルー中の特異点通過）対策・案A
--------------------------------------------------------------------------
[追加K] psi1 スルー中の tau_min ガードと分枝リトライ。

        残課題B: 開始形態が直線近傍（q1≈0）だと、psi1 を固定角へスルーする
        大回転（例: +1.43 -> -0.30 で 1.73 rad）の途中で特異形態を踏み、
        joint を 1 つも動かさないうちに tau_min=0 で abort していた。

        案A（psi1 スルー前にも joint2,3 を開く）は rev.5 の PREP で既に
        行っているが、それだけでは直線近傍からの psi1 大回転を救えなかった。
        そこで本 rev では次を追加する:

        (K-1) スルー中の tau_min を SLEW_FC_T_MIN_MIN で監視し、下回ったら
              「そのスルー経路は特異形態を踏む」と判定して即座に中断する
              （0 になるまで待たない）。

        (K-2) 中断したら、もう一方の分枝（a<->b）へ目標を切り替えて
              スルーをやり直す（rev.18では候補が1つしかないため、この
              リトライは実質発動しない。branch_cand.keys()から動的に
              残り候補を計算する形に修正したので、KeyErrorにはならない）。

        (K-3) 候補を使い切った場合は abort する。

--------------------------------------------------------------------------
継続している変更（rev.5 まで）
--------------------------------------------------------------------------
[追加J] 特異点通過の事前準備 PREP（joint1 が危険帯を通るとき joint2,3 を展開）。
[変更I] 変形順序 joint1 -> joint2,3。
[変更D/追加E/修正F] psi1 解放と GIMBAL_RELEASE、再送判定 self.fix_active。
[修正A/追加C] 固定完了判定、sweep モード。

--------------------------------------------------------------------------
既知の残課題
--------------------------------------------------------------------------
残課題A: 目標の joint3 が符号反転すると joint2,3 畳み直しで特異点を踏む。
         （例: -0.9 0.3 -0.3）。未対策。
既知の制限: controller1停止後もGazebo上でeffortが完全に0にならない
         ことがある（ros_control/gazebo_ros_controlの仕様上の挙動と
         推定、研究ノート参照）。

使用例:
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py -0.9 0.3 0.3
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py --sweep
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from enum import Enum
from controller_manager_msgs.srv import SwitchController


class Step(Enum):
    INIT               = 0
    PREP_JOINT23       = 1
    PREP_STABILIZE     = 2
    GIMBAL_FIX         = 3
    GIMBAL_STABILIZE   = 4
    JOINT1_SERVO       = 5
    JOINT1_STABILIZE   = 6
    GIMBAL_RELEASE     = 7
    JOINT23_SERVO      = 8
    JOINT23_STABILIZE  = 9
    COMPLETE           = 10
    SWEEP              = 11
    JOINT1_TRY1        = 12
    ASK_TRY1           = 13  # 【rev.18追加】try1を実行するか選択させる待機ステップ
    ASK_TRY1_DIRECTION = 14  # 【rev.18追加】try1の変形方向(a/b)を選択させる待機ステップ


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.3
GIMBAL1_SIGN = +1.0

# 分枝の強制。None なら nearest から開始し、失敗したら逆枝へ自動リトライ。
# 【rev.18】_compute_branches/_order_branchesは現在呼ばれていないため、
#          この定数も実質未使用（過去のロジックを削除せず残しているだけ）。
GIMBAL1_FORCE_BRANCH = None

# ---- 特異点通過の事前準備 ---------------------------------------------------
DANGER      = 0.25     # [rad] 危険帯 [-DANGER, +DANGER]
PREP_ANGLE  = 1.2      # [rad] 準備時に joint2,3 を展開する角度
# ---------------------------------------------------------------------------

# ---- rev.6 [追加K]: スルー中の tau_min ガードと分枝リトライ ------------------
SLEW_FC_T_MIN_MIN = 1.0   # [Nm] スルー中これを下回ったら「経路が特異点を踏む」と判定し中断
                          #      0 になるまで待たず早めに切り替える。谷の深さに応じて調整。
SLEW_GUARD_MIN_TRAVEL = 0.15  # [rad] スルー開始直後の過渡を無視するための最小移動量
# ---------------------------------------------------------------------------

GIMBAL_ERR_THRESH = 0.02
FC_T_MIN_REQUIRED = 0.01   # 固定完了後（joint1 変形中）のガード。実機では 1.5~2.0 に

ANGLE_ERROR_THRESHOLD = 0.03
JOINT_RAMP_RATE = 0.0125

STABILIZE_VEL_THRESH = 0.01
STABILIZE_HOLD_LOOPS = 20
STABILIZE_MIN_WAIT   = 1.0
STABILIZE_TIMEOUT    = 6.0

GIMBAL_SLEW_TIMEOUT = 20.0

RELEASE_PSI_RATE_THRESH = 0.01
RELEASE_HOLD_LOOPS      = 20
RELEASE_MIN_WAIT        = 1.5
RELEASE_TIMEOUT         = 12.0
RELEASE_FC_T_MIN_OK     = 2.0

SWEEP_RATE = 0.02

LOOP_FREQ = 20.0
DT = 1.0 / LOOP_FREQ

JOINT1_CONTROLLER = "/hydrus_xi/servo_controller/joints/controller1/simulation"

# ---- controller1停止直後にeffortコマンドをゼロに上書きするためのトピック名 ----
#   ros_control の一般的な仕様として、switch_controller で controller を
#   stop しても、Gazebo側のeffortコマンドバッファには最後の値が残ったまま
#   になる（stopは「新しい値を書き込むのをやめる」だけで、バッファの
#   ゼロクリアは行わない）ことが実測で確認されている。
JOINT1_CMD_TOPIC = JOINT1_CONTROLLER + "/command"
# ---------------------------------------------------------------------------

# ---- 【rev.22追加、rev.23で回数調整】controller1停止前後のゼロpublish回数 ---
#   基準の25回に、念のため安全マージンを乗せて35回とした。
JOINT1_ZERO_PUBLISH_COUNT = 35     # [回] 停止前・停止後それぞれで送る回数
JOINT1_ZERO_PUBLISH_INTERVAL = 0.01  # [s] publishの間隔（10ms）
# ---------------------------------------------------------------------------

# ---- joint2,3 の逐次実行用タイムアウト --------------------------------------
JOINT23_PHASE_TIMEOUT = 15.0  # [s] joint2 がこの時間内に到達しなければ joint3 へ移行
# ---------------------------------------------------------------------------

# ---- 【rev.18追加】gimbal1固定角の新ルールで使う定数 -------------------------
GIMBAL1_PICK_OFFSET = 0.7  # [rad] pi からのオフセット。a>=0でpi+0.7、a<0でpi-0.7
# ---------------------------------------------------------------------------


class GimbalFixedSequencer(object):

    def __init__(self, q1, q2, q3, sweep=False):
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.target_q = {'joint1': q1, 'joint2': q2, 'joint3': q3}

        self.current_q  = {n: 0.0 for n in self.joint_names}
        self.current_dq = {n: 0.0 for n in self.joint_names}
        self.joint_targets = {n: 0.0 for n in self.joint_names}

        self.fix_enabled = False
        self.fix_target  = 0.0
        self.fix_current = 0.0
        self.fc_t_min    = 0.0
        self.fix_err     = math.pi
        self.fix_state_received = False

        self.fix_active = False

        self.sweep_mode  = sweep
        self.sweep_cmd   = 0.0
        self.sweep_travel = 0.0

        self.gimbal1_cmd = 0.0
        self.step = Step.INIT
        self.step_t0 = None
        self.hold_count = 0
        self.aborted = False

        self.prep_used = False

        self.release_hold = 0
        self.psi1_prev = None

        # rev.6 [追加K]: 分枝リトライ用（rev.18では候補は基本1つのみ）
        self.branch_cand = {}       # {'a': val} など
        self.branch_tried = []      # 試した分枝キー
        self.branch_current = None  # 現在試している分枝キー
        self.slew_start_psi = None  # スルー開始時の psi1（過渡判定用）

        # 【rev.17追加】joint2,3 逐次実行用フェーズ（0: joint2, 1: joint3）
        self._joint23_phase = 0
        self._joint23_phase_t0 = rospy.Time.now()

        # 【rev.18追加】try1で使うgimbal1目標角（ASK_TRY1_DIRECTIONで設定される）
        self._try1_gimbal_target = None

        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.fix_cmd_pub     = rospy.Publisher('/hydrus_xi/fixed_gimbal_cmd', Float64MultiArray, queue_size=1)
        # controller1停止直後にeffortコマンドをゼロへ上書きするための Publisher
        self.joint1_cmd_zero_pub = rospy.Publisher(JOINT1_CMD_TOPIC, Float64, queue_size=1)

        self.switch_ctrl = rospy.ServiceProxy(
            '/hydrus_xi/controller_manager/switch_controller', SwitchController)

        rospy.Subscriber('/hydrus_xi/joint_states', JointState, self._joint_state_cb)
        rospy.Subscriber('/hydrus_xi/fixed_gimbal_state', Float64MultiArray, self._fix_state_cb)

        rospy.loginfo("[Seq] waiting for /hydrus_xi/joint_states ...")
        try:
            msg = rospy.wait_for_message('/hydrus_xi/joint_states', JointState, timeout=5.0)
            self._joint_state_cb(msg)
        except rospy.ROSException:
            rospy.logwarn("[Seq] joint_states timeout. starting from 0.0")

        rospy.loginfo("[Seq] waiting for /hydrus_xi/fixed_gimbal_state ...")
        try:
            msg = rospy.wait_for_message('/hydrus_xi/fixed_gimbal_state', Float64MultiArray, timeout=10.0)
            self._fix_state_cb(msg)
        except rospy.ROSException:
            rospy.logerr("[Seq] no fixed_gimbal_state. is the modified navigator running?")

        for n in self.joint_names:
            self.joint_targets[n] = self.current_q[n]

        rate = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate.sleep()

        self._send_joint_cmd()
        self._release_fix()

        if self.sweep_mode:
            rospy.logwarn("[Seq] === SWEEP MODE === joints are held. fc_t_min guard disabled.")
            self.sweep_cmd = self.fix_current
            self._goto(Step.SWEEP)
        else:
            rospy.loginfo("[Seq] init. q=(%.3f, %.3f, %.3f) -> target=(%.3f, %.3f, %.3f), psi1=%+.3f",
                          self.current_q['joint1'], self.current_q['joint2'], self.current_q['joint3'],
                          q1, q2, q3, self.fix_current)

        self.timer = rospy.Timer(rospy.Duration(DT), self._loop)

    # ---------------- callbacks ----------------

    def _joint_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_q:
                self.current_q[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.current_dq[name] = msg.velocity[i]

    def _fix_state_cb(self, msg):
        if len(msg.data) < 5:
            return
        self.fix_enabled = (msg.data[0] > 0.5)
        self.fix_target  = msg.data[1]
        self.fix_current = msg.data[2]
        self.fc_t_min    = msg.data[3]
        self.fix_err     = msg.data[4]
        self.fix_state_received = True

    # ---------------- helpers ----------------

    @staticmethod
    def _norm(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _diff(self, joint):
        return self._norm(self.target_q[joint] - self.current_q[joint])

    def _send_joint_cmd(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for n in self.joint_names:
            msg.name.append(n)
            msg.position.append(float(self.joint_targets[n]))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
        self.joints_ctrl_pub.publish(msg)

    def _send_fix_cmd(self, enable, angle):
        msg = Float64MultiArray()
        msg.data = [1.0 if enable else 0.0, float(angle)]
        self.fix_cmd_pub.publish(msg)

    def _hold_fix(self):
        self.fix_active = True
        self._send_fix_cmd(True, self.gimbal1_cmd)

    def _release_fix(self):
        self.fix_active = False
        self._send_fix_cmd(False, 0.0)

    def _goto(self, step):
        self.step = step
        self.step_t0 = rospy.Time.now()
        self.hold_count = 0
        self.release_hold = 0
        self.psi1_prev = None
        if step == Step.JOINT23_SERVO:
            # joint2,3のステップに入るたびにフェーズを joint2 から再開する
            self._joint23_phase = 0
            self._joint23_phase_t0 = rospy.Time.now()

    def _elapsed(self):
        return (rospy.Time.now() - self.step_t0).to_sec()

    def _settled(self, joints):
        if self._elapsed() < STABILIZE_MIN_WAIT:
            return False
        if all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in joints):
            self.hold_count += 1
        else:
            self.hold_count = 0
        if self._elapsed() >= STABILIZE_TIMEOUT:
            rospy.logwarn("[Seq] settle timeout (%.1f s). proceeding anyway.", STABILIZE_TIMEOUT)
            return True
        return self.hold_count >= STABILIZE_HOLD_LOOPS

    def _ramp(self, joints):
        for j in joints:
            d = self._norm(self.target_q[j] - self.joint_targets[j])
            if abs(d) > JOINT_RAMP_RATE:
                self.joint_targets[j] += math.copysign(JOINT_RAMP_RATE, d)
            else:
                self.joint_targets[j] = self.target_q[j]

    def _ramp_to(self, joints, goals):
        done = True
        for j, g in zip(joints, goals):
            d = self._norm(g - self.joint_targets[j])
            if abs(d) > JOINT_RAMP_RATE:
                self.joint_targets[j] += math.copysign(JOINT_RAMP_RATE, d)
                done = False
            else:
                self.joint_targets[j] = g
            if abs(self._norm(g - self.current_q[j])) > ANGLE_ERROR_THRESHOLD:
                done = False
        return done

    def _reached(self, joints):
        return all(abs(self._diff(j)) <= ANGLE_ERROR_THRESHOLD for j in joints)

    def _joint1_crosses_danger(self):
        lo, hi = sorted([self.current_q['joint1'], self.target_q['joint1']])
        return (lo < DANGER) and (hi > -DANGER)

    # ---- rev.6 [変更L]: 両分枝の候補を作る（rev.18時点では未使用、参考のため残置） ----
    def _compute_branches(self, sign):
        """
        【rev.18: 現在このメソッドは呼ばれていない】
        sign から a, b の 2 候補を計算して dict で返す。
          a = -MAG*SIGN*sign,  b = pi - a   （sin が等しく joint1 モーメント同一）
        """
        a = self._norm(-GIMBAL1_MAG * GIMBAL1_SIGN * sign)
        b = self._norm(math.pi - a)
        return {'a': a, 'b': b}

    def _order_branches(self, cand):
        """
        【rev.18: 現在このメソッドは呼ばれていない】
        試す順序を決める。FORCE 指定があればそれのみ。
        なければ「現在角に近い方」を先に、遠い方を後に。
        """
        if GIMBAL1_FORCE_BRANCH in ('a', 'b'):
            return [GIMBAL1_FORCE_BRANCH]
        psi_now = self.fix_current
        da = abs(self._norm(cand['a'] - psi_now))
        db = abs(self._norm(cand['b'] - psi_now))
        return ['a', 'b'] if da <= db else ['b', 'a']

    # ---- 【rev.18追加、rev.20で符号修正】gimbal1固定角の新ルール ------------
    def _pick_gimbal1_target(self):
        """
        joint1を動かす前に、gimbal1の固定角を以下のルールで1つだけ決める。
          remainder = 現在のjoint1角度 mod pi(3.14)
          a = 目標のjoint1角度 - remainder
          a が正 -> gimbal1 = pi - 0.7 （b型）
          a が負 -> gimbal1 = pi + 0.7 （a型）

        【rev.20修正】try1でa/bを両方実際に試した結果、
          「目標角-現在角が正のときはb型（-0.7側）、負のときはa型（+0.7側）」
          が正しい対応であることが確認された。rev.18時点ではこの分岐が
          逆（正->a型、負->b型）になっていたため、if/elseの中身を入れ替えた。
          _pick_try1_targetのa/b公式自体（a:+0.7, b:-0.7）は変更していない。
        """
        remainder = self.current_q['joint1'] % math.pi
        rospy.loginfo("[Seq] joint1 gimbal-pick: current=%.4f rad, current mod pi = %.4f rad",
                      self.current_q['joint1'], remainder)

        a = self.target_q['joint1'] - remainder
        if a >= 0:
            target = self._norm(math.pi - GIMBAL1_PICK_OFFSET)
            rospy.loginfo("[Seq] joint1 gimbal-pick: a = target(%.4f) - remainder(%.4f) = %+.4f (>=0) "
                          "-> gimbal1 = pi - %.1f (b-type) = %+.3f rad",
                          self.target_q['joint1'], remainder, a, GIMBAL1_PICK_OFFSET, target)
        else:
            target = self._norm(math.pi + GIMBAL1_PICK_OFFSET)
            rospy.loginfo("[Seq] joint1 gimbal-pick: a = target(%.4f) - remainder(%.4f) = %+.4f (<0) "
                          "-> gimbal1 = pi + %.1f (a-type) = %+.3f rad",
                          self.target_q['joint1'], remainder, a, GIMBAL1_PICK_OFFSET, target)
        return target

    # ---- 【rev.19追加】try1用: 現在のgimbal1角度を基準にした目標角の決定 -------
    def _pick_try1_target(self, direction):
        """
        try1でgimbal1を固定する目標角を、現在のgimbal1角度(self.fix_current)
        を基準に決める。

        joint1変形前(_pick_gimbal1_target)と違い、try1に入る直前の
        gimbal1角度はJOINT23_SERVO中の自由最適化の結果次第で毎回不定になる。
        そこで、固定の"pi+0.7"/"pi-0.7"ではなく、現在角bをpiで割った商cを
        求め、その"pi*cの帯"を基準にオフセットする。これにより、a/bという
        物理的な向きの意味は変えずに、現在角から近い目標角を毎回選び直せる。

          b = 現在のgimbal1角度 (self.fix_current)
          b ÷ pi の商を c、余りを d とする (b = c*pi + d)
          direction='a' -> target = c*pi + 0.7
          direction='b' -> target = c*pi - 0.7

        戻り値: (target, c, d) のタプル。b, c, d, target はすべて呼び出し
        元で表示する（今回の変更の目的そのものであるため）。
        """
        b = self.fix_current
        c, d = divmod(b, math.pi)

        if direction == 'a':
            target = self._norm(c * math.pi + GIMBAL1_PICK_OFFSET)
        elif direction == 'b':
            target = self._norm(c * math.pi - GIMBAL1_PICK_OFFSET)
        else:
            raise ValueError("direction must be 'a' or 'b', got %r" % direction)

        rospy.loginfo("[Seq] try1 gimbal-pick: b(current gimbal1)=%.4f, "
                      "b/pi -> c=%.1f, d(remainder)=%.4f (b = c*pi + d)",
                      b, c, d)
        rospy.loginfo("[Seq] try1 gimbal-pick: direction='%s' -> target = c*pi %s %.1f = %+.3f rad",
                      direction, '+' if direction == 'a' else '-', GIMBAL1_PICK_OFFSET, target)

        return target, c, d

    def _check_fc_t_min(self):
        """固定完了後（joint1 変形中）の tau_min ガード"""
        if self.sweep_mode:
            return True
        if not (self.fix_active and self.fix_enabled) or not self.fix_state_received:
            return True
        if self.fc_t_min < FC_T_MIN_REQUIRED:
            rospy.logerr("[Seq] ABORT: fc_t_min = %.3f Nm < %.3f (psi1=%+.3f, q1=%+.4f). gimbal fix released.",
                         self.fc_t_min, FC_T_MIN_REQUIRED, self.fix_current, self.current_q['joint1'])
            self._release_fix()
            self.aborted = True
            self._goto(Step.COMPLETE)
            return False
        return True

    # ---------------- steps ----------------

    def _step_init(self):
        """(1) 初期静定 -> 危険帯判定して分岐"""
        self._send_joint_cmd()
        self._release_fix()

        if not self.fix_state_received:
            rospy.logwarn_throttle(2.0, "[Seq] waiting for fixed_gimbal_state ...")
            return

        if self._settled(self.joint_names):
            d1 = self._diff('joint1')
            if abs(d1) <= ANGLE_ERROR_THRESHOLD:
                rospy.loginfo("[Seq] joint1 already at target -> joint2,3 (psi1 free)")
                self._goto(Step.JOINT23_SERVO)
                return

            if self._joint1_crosses_danger():
                self.prep_used = True
                rospy.loginfo("[Seq] joint1 crosses danger band [-%.2f, +%.2f] "
                              "(start=%.3f -> target=%.3f) -> PREP: expand joint2,3 to %.2f",
                              DANGER, DANGER, self.current_q['joint1'],
                              self.target_q['joint1'], PREP_ANGLE)
                self._goto(Step.PREP_JOINT23)
            else:
                self.prep_used = False
                rospy.loginfo("[Seq] joint1 stays clear of danger band -> gimbal fix directly")
                self._enter_gimbal_fix(d1)

    def _step_prep_joint23(self):
        """(P1) joint2,3 を PREP_ANGLE へ展開（psi1 自由）"""
        self._release_fix()
        done = self._ramp_to(['joint2', 'joint3'], [PREP_ANGLE, PREP_ANGLE])
        self._send_joint_cmd()

        rospy.loginfo_throttle(0.5, "[Seq] PREP: joint2,3 -> %.2f | q=(%.4f, %.4f) fc_t_min=%.3f",
                               PREP_ANGLE, self.current_q['joint2'], self.current_q['joint3'], self.fc_t_min)

        if done:
            rospy.loginfo("[Seq] PREP joint2,3 expanded -> stabilize")
            self._goto(Step.PREP_STABILIZE)

    def _step_prep_stabilize(self):
        """(P2) 準備後の静定 -> gimbal 固定へ"""
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] PREP settled (fc_t_min=%.3f) -> gimbal fix", self.fc_t_min)
            self._enter_gimbal_fix(self._diff('joint1'))

    def _enter_gimbal_fix(self, d1):
        """
        【rev.18変更】gimbal1の固定角を、新ルール（_pick_gimbal1_target）で
        1つだけ決めてスルー開始する。旧来の2分岐リトライの枠組み
        （branch_cand/branch_tried/_start_branch）はそのまま流用するが、
        候補は1つ（キー'a'）のみになる。
        """
        target = self._pick_gimbal1_target()
        self.branch_cand = {'a': target}
        self.branch_tried = []
        self._start_branch('a', d1)

    def _start_branch(self, key, d1=None):
        """指定分枝でスルーを開始する"""
        self.branch_current = key
        self.branch_tried.append(key)
        self.gimbal1_cmd = self.branch_cand[key]
        self.slew_start_psi = self.fix_current
        rospy.loginfo("[Seq] branch '%s': gimbal1 target = %+.3f rad (psi_now=%+.3f)",
                      key, self.gimbal1_cmd, self.fix_current)
        self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        """(2) gimbal1 を固定角へスルー。rev.6: スルー中 tau_min を監視し、
              谷を踏んだら別候補へリトライする（rev.18では候補が1つのため、
              実質的にはリトライ先がなくそのままABORTする）。"""
        self._send_joint_cmd()
        self._hold_fix()

        # スルー開始からの移動量（過渡の除外用）
        traveled = abs(self._norm(self.fix_current - (self.slew_start_psi or self.fix_current)))

        # [追加K-1] スルー中の tau_min 監視（過渡を過ぎてから）
        if (self.fix_active and self.fix_enabled and self.fix_state_received
                and traveled > SLEW_GUARD_MIN_TRAVEL
                and self.fc_t_min < SLEW_FC_T_MIN_MIN):
            rospy.logwarn("[Seq] branch '%s' slew hits low tau_min=%.3f at psi1=%+.3f "
                          "(< %.2f). this path crosses singularity.",
                          self.branch_current, self.fc_t_min, self.fix_current, SLEW_FC_T_MIN_MIN)
            # [追加K-2] 他に試していない候補があれば切り替える
            # 【rev.18】候補キーを ('a','b') 固定ではなく branch_cand.keys() から
            #          動的に求める（候補が1つしかない場合にKeyErrorしないため）。
            remaining = [k for k in self.branch_cand.keys() if k not in self.branch_tried]
            if remaining:
                rospy.loginfo("[Seq] retrying with the other candidate '%s'", remaining[0])
                self._release_fix()   # 一度解放して psi1 を自由に戻す
                self._start_branch(remaining[0])
                return
            else:
                # [追加K-3] 候補を使い切った -> この開始形態からは固定変形不能
                rospy.logerr("[Seq] ABORT: gimbal1 slew crosses singularity and no "
                             "alternative target remains (q1_start=%+.3f). "
                             "gimbal-fixed joint1 deform is infeasible from this "
                             "near-singular start.", self.current_q['joint1'])
                self._release_fix()
                self.aborted = True
                self._goto(Step.COMPLETE)
                return

        rospy.loginfo_throttle(0.5, "[Seq] slewing[%s]: en=%d cur=%+.3f tgt=%+.3f err=%.4f fc_t_min=%.3f travel=%.2f",
                               self.branch_current, self.fix_enabled, self.fix_current, self.fix_target,
                               self.fix_err, self.fc_t_min, traveled)

        if self.fix_enabled and self.fix_err <= GIMBAL_ERR_THRESH:
            rospy.loginfo("[Seq] gimbal1 fixed at %+.3f rad (branch '%s', fc_t_min=%.3f Nm) -> stabilize",
                          self.fix_current, self.branch_current, self.fc_t_min)
            self._goto(Step.GIMBAL_STABILIZE)
        elif self._elapsed() > GIMBAL_SLEW_TIMEOUT:
            rospy.logerr("[Seq] ABORT: gimbal1 slew timeout (branch '%s' en=%d err=%.4f)",
                         self.branch_current, self.fix_enabled, self.fix_err)
            self._release_fix()
            self.aborted = True
            self._goto(Step.COMPLETE)

    def _step_gimbal_stabilize(self):
        """(3) 固定後の静定"""
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] stabilized (fc_t_min=%.3f) -> joint1 servo deform", self.fc_t_min)
            self._goto(Step.JOINT1_SERVO)

    def _step_joint1_servo(self):
        """(4) joint1 をサーボで変形（gimbal1 固定）"""
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        self._ramp(['joint1'])
        self._send_joint_cmd()

        rospy.loginfo_throttle(0.5, "[Seq] joint1: q=%.4f tgt=%.4f cmd=%.4f fc_t_min=%.3f",
                               self.current_q['joint1'], self.target_q['joint1'],
                               self.joint_targets['joint1'], self.fc_t_min)

        if self._reached(['joint1']):
            rospy.loginfo("[Seq] joint1 deform done (q1=%.4f) -> stabilize", self.current_q['joint1'])
            self._goto(Step.JOINT1_STABILIZE)

    def _step_joint1_stabilize(self):
        """(5) joint1 変形終了・静定 -> psi1 解放"""
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(['joint1']):
            rospy.loginfo("[Seq] joint1 settled (fc_t_min=%.3f) -> release psi1", self.fc_t_min)
            self._release_fix()
            self._goto(Step.GIMBAL_RELEASE)

    def _step_gimbal_release(self):
        """(6) psi1 を最適化に返し、収束を待つ"""
        self._send_joint_cmd()
        self._release_fix()

        psi1 = self.fix_current
        if self.psi1_prev is None:
            self.psi1_prev = psi1
        d_psi1 = abs(self._norm(psi1 - self.psi1_prev))
        self.psi1_prev = psi1

        if self.fix_enabled:
            self.release_hold = 0
            rospy.logwarn_throttle(1.0, "[Seq] waiting for navigator to release psi1 ...")
        else:
            quiet_psi   = (d_psi1 < RELEASE_PSI_RATE_THRESH)
            quiet_joint = all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in self.joint_names)
            torque_ok   = (self.fc_t_min >= RELEASE_FC_T_MIN_OK)
            if quiet_psi and quiet_joint and torque_ok:
                self.release_hold += 1
            else:
                self.release_hold = 0

        rospy.loginfo_throttle(0.5, "[Seq] release: en=%d psi1=%+.4f dpsi=%.4f fc_t_min=%.3f hold=%d/%d",
                               self.fix_enabled, psi1, d_psi1, self.fc_t_min,
                               self.release_hold, RELEASE_HOLD_LOOPS)

        if self._elapsed() < RELEASE_MIN_WAIT:
            return

        if self.release_hold >= RELEASE_HOLD_LOOPS:
            rospy.loginfo("[Seq] psi1 released and settled at %+.3f rad (fc_t_min=%.3f Nm) -> joint2,3",
                          psi1, self.fc_t_min)
            self._goto(Step.JOINT23_SERVO)
            return

        if self._elapsed() >= RELEASE_TIMEOUT:
            rospy.logwarn("[Seq] release settle timeout (%.1f s): psi1=%+.4f dpsi=%.4f fc_t_min=%.3f. "
                          "proceeding to joint2,3 anyway.",
                          RELEASE_TIMEOUT, psi1, d_psi1, self.fc_t_min)
            self._goto(Step.JOINT23_SERVO)

    def _step_joint23_servo(self):
        """(7) joint2,3 を最終目標角へ変形（psi1 自由）。
        joint2 -> joint3 の順に逐次実行する。joint2 が
        JOINT23_PHASE_TIMEOUT 秒経っても目標に到達しなければ、
        joint3 の動作へ移行する。"""
        self._release_fix()

        if self._joint23_phase == 0:
            self._ramp(['joint2'])
            self._send_joint_cmd()

            rospy.loginfo_throttle(0.5, "[Seq] joint2,3: [joint2] q=(%.4f, %.4f) tgt=(%.4f, %.4f) psi1=%+.3f fc_t_min=%.3f",
                                   self.current_q['joint2'], self.current_q['joint3'],
                                   self.target_q['joint2'], self.target_q['joint3'],
                                   self.fix_current, self.fc_t_min)

            if abs(self._diff('joint2')) <= ANGLE_ERROR_THRESHOLD:
                rospy.loginfo("[Seq] joint2 deform done (q2=%.4f) -> joint3", self.current_q['joint2'])
                self._joint23_phase = 1
                self._joint23_phase_t0 = rospy.Time.now()
            elif (rospy.Time.now() - self._joint23_phase_t0).to_sec() >= JOINT23_PHASE_TIMEOUT:
                rospy.logwarn("[Seq] joint2 phase timeout (%.1fs): q2=%.4f tgt=%.4f. proceeding to joint3.",
                              JOINT23_PHASE_TIMEOUT, self.current_q['joint2'], self.target_q['joint2'])
                self._joint23_phase = 1
                self._joint23_phase_t0 = rospy.Time.now()
        else:
            self._ramp(['joint3'])
            self._send_joint_cmd()

            rospy.loginfo_throttle(0.5, "[Seq] joint2,3: [joint3] q=(%.4f, %.4f) tgt=(%.4f, %.4f) psi1=%+.3f fc_t_min=%.3f",
                                   self.current_q['joint2'], self.current_q['joint3'],
                                   self.target_q['joint2'], self.target_q['joint3'],
                                   self.fix_current, self.fc_t_min)

            if self._reached(['joint2', 'joint3']):
                rospy.loginfo("[Seq] joint2,3 deform done -> stabilize")
                self._goto(Step.JOINT23_STABILIZE)

    def _step_joint23_stabilize(self):
        """(8) 最終静定 -> try1を実行するか選択させる"""
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(['joint2', 'joint3']):
            rospy.loginfo("[Seq] all settled -> ask whether to run joint1_try1")
            self._goto(Step.ASK_TRY1)

    def _step_ask_try1(self):
        """
        【rev.18追加】待機ステップ。実際の「1/2」入力受付はmain()の
        対話プロンプトで行う。ここではjoint/psi1の状態を保持するだけ。
        """
        self._send_joint_cmd()
        self._release_fix()

    def _step_ask_try1_direction(self):
        """
        【rev.18追加】待機ステップ。実際の「a/b」入力受付はmain()の
        対話プロンプトで行う。ここではjoint/psi1の状態を保持するだけ。
        """
        self._send_joint_cmd()
        self._release_fix()

    def start_try1(self, gimbal_target):
        """
        【rev.18追加】main()の対話プロンプトで方向(a/b)が選ばれた際に
        呼ばれる。gimbal1の目標角を確定し、JOINT1_TRY1へ遷移する。
        """
        self._try1_gimbal_target = gimbal_target
        for attr in ('_try1_reached_t', '_try1_stopped'):
            if hasattr(self, attr):
                delattr(self, attr)
        self._goto(Step.JOINT1_TRY1)

    def _step_joint1_try1(self):
        """
        【rev.18変更】全周スピン診断は削除。start_try1()で確定した
        gimbal1目標角（pi+0.7 または pi-0.7）へ直接向かい、到達・静定後に
        controller1を停止し、effortゼロのpublishを試みる。
        """
        gimbal_angle = self._try1_gimbal_target
        if gimbal_angle is None:
            rospy.logerr_throttle(1.0, "[Seq] joint1_try1: _try1_gimbal_target is not set "
                                  "(start_try1() was not called). aborting this step.")
            return

        # (1) gimbal1 を固定
        self.gimbal1_cmd = gimbal_angle
        self._hold_fix()

        # (2) gimbal1 が目標に届くまで待つ
        if abs(self._norm(self.fix_current - gimbal_angle)) > 0.05:
            rospy.loginfo_throttle(0.5, "[Seq] joint1_try1: waiting gimbal1 -> %+.3f (now %+.3f)",
                                   gimbal_angle, self.fix_current)
            return

        # (2.5) 到達したら、その時刻を記録して安定化の待ちを始める
        if not getattr(self, '_try1_reached_t', None):
            self._try1_reached_t = rospy.Time.now()
            rospy.loginfo("[Seq] joint1_try1: gimbal1 reached %+.3f, waiting to settle", self.fix_current)

        # (2.6) 機体が安定するまで待つ（関節速度が十分小さくなるまで）
        settled = all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in self.joint_names)
        waited = (rospy.Time.now() - self._try1_reached_t).to_sec()
        if not settled and waited < 6.0:   # 安定するか、最大6秒待つ
            rospy.loginfo_throttle(0.5, "[Seq] joint1_try1: settling... (%.1fs)", waited)
            return

        # (3) 安定したら joint1 のサーボを切る（1回だけ）
        if not getattr(self, '_try1_stopped', False):
            # 【rev.22追加、rev.23で回数調整】GitHub Copilotの分析を踏まえた対策。
            # ros_control/gazebo_ros_controlは、controllerをstopしても
            # Gazebo側のeffortコマンドバッファを自動でクリアしない
            # （最後に書き込まれた値が残り続ける）。1回のpublishだと
            # 受信タイミングによって取りこぼすことがあるため、停止の
            # 前後でそれぞれ複数回、10ms間隔でeffort=0を送り、バッファを
            # 確実に上書きする。
            #
            # 回数は基準の25回に、念のため安全マージンを乗せて
            # JOINT1_ZERO_PUBLISH_COUNT=35回とした（追加コストは
            # 10ms×35×2≈0.7秒、停止処理はシーケンス全体で1回きりなので
            # 実害は小さいと判断）。
            #
            # 注意: このブロックは20Hzのタイマーコールバック内で
            # 約0.7秒ブロックする。停止処理はシーケンス全体で1回きり
            # なので実害は小さいと判断し、そのまま実装する。
            for _ in range(JOINT1_ZERO_PUBLISH_COUNT):
                self.joint1_cmd_zero_pub.publish(Float64(0.0))
                rospy.sleep(JOINT1_ZERO_PUBLISH_INTERVAL)

            self.switch_ctrl(start_controllers=[],
                             stop_controllers=[JOINT1_CONTROLLER],
                             strictness=1)
            rospy.loginfo("[Seq] joint1_try1: settled, controller1 stopped")
            self._try1_stopped = True

            for _ in range(JOINT1_ZERO_PUBLISH_COUNT):
                self.joint1_cmd_zero_pub.publish(Float64(0.0))
                rospy.sleep(JOINT1_ZERO_PUBLISH_INTERVAL)
            rospy.loginfo("[Seq] joint1_try1: published effort=0.0 x%d(before) + x%d(after) to %s",
                          JOINT1_ZERO_PUBLISH_COUNT, JOINT1_ZERO_PUBLISH_COUNT, JOINT1_CMD_TOPIC)
        # 以降このステップに留まり、_send_joint_cmd() を呼ばない。

    def _step_complete(self):
        """完了。gimbal1 は自由のまま"""
        self._send_joint_cmd()
        self._release_fix()

        if self._elapsed() < 0.1:
            tag = "ABORTED" if self.aborted else "DONE"
            rospy.loginfo("[Seq] %s%s  q=(%.4f/%.4f, %.4f/%.4f, %.4f/%.4f)  psi1=%+.3f  fc_t_min=%.3f",
                          tag, " (prep used)" if self.prep_used else "",
                          self.current_q['joint1'], self.target_q['joint1'],
                          self.current_q['joint2'], self.target_q['joint2'],
                          self.current_q['joint3'], self.target_q['joint3'],
                          self.fix_current, self.fc_t_min)

    # ---- sweep モード ----
    def _step_sweep(self):
        self._send_joint_cmd()
        self.sweep_cmd = self._norm(self.sweep_cmd + SWEEP_RATE)
        self.sweep_travel += SWEEP_RATE
        self.fix_active = True
        self._send_fix_cmd(True, self.sweep_cmd)

        rospy.loginfo_throttle(0.25, "[Sweep] psi1=%+.4f  fc_t_min=%.4f  travel=%.2f/%.2f rad",
                               self.fix_current, self.fc_t_min, self.sweep_travel, 2 * math.pi)

        if self.sweep_travel >= 2 * math.pi:
            rospy.loginfo("[Sweep] one full revolution done. releasing gimbal fix.")
            self._release_fix()
            self._goto(Step.COMPLETE)

    # ---------------- main loop ----------------

    def _loop(self, _event):
        try:
            if rospy.Time.now().is_zero():
                return
            if self.step_t0 is None:
                self.step_t0 = rospy.Time.now()

            {
                Step.INIT:               self._step_init,
                Step.PREP_JOINT23:       self._step_prep_joint23,
                Step.PREP_STABILIZE:     self._step_prep_stabilize,
                Step.GIMBAL_FIX:         self._step_gimbal_fix,
                Step.GIMBAL_STABILIZE:   self._step_gimbal_stabilize,
                Step.JOINT1_SERVO:       self._step_joint1_servo,
                Step.JOINT1_STABILIZE:   self._step_joint1_stabilize,
                Step.GIMBAL_RELEASE:     self._step_gimbal_release,
                Step.JOINT23_SERVO:      self._step_joint23_servo,
                Step.JOINT23_STABILIZE:  self._step_joint23_stabilize,
                Step.COMPLETE:           self._step_complete,
                Step.SWEEP:              self._step_sweep,
                Step.JOINT1_TRY1:        self._step_joint1_try1,
                Step.ASK_TRY1:           self._step_ask_try1,
                Step.ASK_TRY1_DIRECTION: self._step_ask_try1_direction,
            }[self.step]()
        except Exception as e:
            rospy.logerr("[Seq] loop error: %s", str(e))

    def new_target(self, q1, q2, q3):
        for n in self.joint_names:
            self.joint_targets[n] = self.current_q[n]
        self._send_joint_cmd()
        self._release_fix()

        self.target_q = {'joint1': q1, 'joint2': q2, 'joint3': q3}
        self.aborted = False
        self.prep_used = False
        self.branch_tried = []
        self.branch_current = None
        self._goto(Step.INIT)
        rospy.loginfo("[Seq] new target: (%.3f, %.3f, %.3f)", q1, q2, q3)

    def shutdown(self):
        self.timer.shutdown()


def main():
    rospy.init_node('hydrus_xi_gimbal_fixed_sequencer')

    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    sweep = '--sweep' in args
    args = [a for a in args if a != '--sweep']

    q = [0.0, 0.0, 0.0]
    if len(args) >= 3:
        q = [float(args[i]) for i in range(3)]

    seq = GimbalFixedSequencer(q[0], q[1], q[2], sweep=sweep)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if seq.step == Step.COMPLETE:
            if sweep:
                rospy.loginfo("[Sweep] finished.")
                break
            print("\n" + "=" * 56)
            print(" Hydrus-Xi : joint1 thrust-deform (prep + slew branch retry)")
            print(" next target [q1 q2 q3]  (q to quit)")
            print("=" * 56)
            try:
                s = input("> ").strip()
                if s.lower() == 'q':
                    break
                v = [float(x) for x in s.split()]
                if len(v) == 3:
                    seq.new_target(*v)
            except (ValueError, KeyboardInterrupt, EOFError):
                break

        elif seq.step == Step.ASK_TRY1:
            print("\n" + "=" * 56)
            print(" joint1_try1 を実行しますか？")
            print(" 1: 実行する")
            print(" 2: スキップする")
            print("=" * 56)
            try:
                s = input("> ").strip()
            except (KeyboardInterrupt, EOFError):
                break
            if s == '1':
                seq._goto(Step.ASK_TRY1_DIRECTION)
            elif s == '2':
                rospy.loginfo("[Seq] joint1_try1 skipped by user")
                seq._goto(Step.COMPLETE)
            else:
                print("'1' か '2' を入力してください")

        elif seq.step == Step.ASK_TRY1_DIRECTION:
            print("\n変形方向は？")
            print(" a: gimbal1 = c*pi + %.1f rad に固定（cは現在角basis）" % GIMBAL1_PICK_OFFSET)
            print(" b: gimbal1 = c*pi - %.1f rad に固定（cは現在角basis）" % GIMBAL1_PICK_OFFSET)
            try:
                s = input("> ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break
            if s == 'a':
                target, c, d = seq._pick_try1_target('a')
                msg = ("[Seq] try1: direction 'a' selected -> b(current)=%.4f, c=%.1f, d=%.4f, "
                       "target = c*pi + %.1f = %+.3f rad" % (seq.fix_current, c, d, GIMBAL1_PICK_OFFSET, target))
                print(msg)
                rospy.loginfo(msg)
                seq.start_try1(target)
            elif s == 'b':
                target, c, d = seq._pick_try1_target('b')
                msg = ("[Seq] try1: direction 'b' selected -> b(current)=%.4f, c=%.1f, d=%.4f, "
                       "target = c*pi - %.1f = %+.3f rad" % (seq.fix_current, c, d, GIMBAL1_PICK_OFFSET, target))
                print(msg)
                rospy.loginfo(msg)
                seq.start_try1(target)
            else:
                print("'a' か 'b' を入力してください")

        else:
            rate.sleep()

    seq.shutdown()


if __name__ == '__main__':
    main()