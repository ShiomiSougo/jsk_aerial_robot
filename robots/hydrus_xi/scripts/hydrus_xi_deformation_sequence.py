#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）rev.15

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま joint1 の変形を成立させる。

--------------------------------------------------------------------------
rev.15 での変更（rev.14 からの差分）— joint2,3変形中の特異点通過ガード追加
--------------------------------------------------------------------------
[追加U] 深刻なバグ修正：_step_joint23_servo に、S1/S2特異条件を横切る
        際の安全ガードが一切存在しなかった。

        発見の経緯: q=(0.5, -1, 0.5) への変形実験でフォースランディングが
        発生した。ログを解析した結果、joint1(=0.527, 固定済み)・
        joint3(=0.499, 目標に先着して停止)がほぼ等しい値のまま、
        joint2 だけが 1.57 -> -1.0 へ単純な直線補間（_ramp）で動かされ、
        その途中で joint2 ≈ -0.5 を通過した瞬間に S2特異条件
        （q1 = -q2 = q3）を踏み抜き、fc_t_min が 0 に落ちて姿勢崩壊、
        フォースランディングに至った。

        根本原因は二重にあった：
          (1) _step_joint23_servo は _ramp() で joint2,3 を固定レート
              (JOINT_RAMP_RATE) で単純に直線補間するだけで、経路上で
              特異条件に近づいていないかを一切確認していなかった。
          (2) _check_fc_t_min() は fix_active（gimbal1固定モード中）
              のときしか働かず、gimbal1解放後（JOINT23_SERVOはまさに
              この状態）では常に無条件でパスするだけの無防備な設計
              だった。

        対策として、gimbal1のrampロジック（rev.11で確立した「危険域は
        素早く通過すべき」という原則）と同じ考え方を joint2,3 の変形
        速度にも適用した。fc_t_min が低下するほど joint2,3 の変化速度
        を自動的に引き上げ（JOINT23_RATE_NORMAL -> JOINT23_RATE_FAST の
        連続補間）、危険域の滞在時間を短縮する。gimbal1側のrampとは
        独立した状態変数（_joint23_ramp_current_rate）を持つ。

        なお、これは対症療法であり、経路そのものが特異点を横切らない
        よう事前に計画する（Zhao et al., 2016のRRT*に相当する経路計画）
        という根本対策は依然として今後の課題として残る。

--------------------------------------------------------------------------
rev.14 での変更（rev.11 からの差分）— joint1_try1 を3点移動方式に変更
--------------------------------------------------------------------------
[追加R] 全周スピンによる特異点診断（旧 _step_joint1_try1）を廃止し、
        「安全域（±1.0rad付近）だけを通る3点移動」に置き換えた。

        背景: 全周スピン方式は、πおよびその近傍（既知の深い特異点帯）
        を毎回横断する設計だった。到達判定が self._spin_travel という
        「コマンドの積算カウンタ」に基づいていたため、fix_gimbal_slew_rate_
        （C++側の物理スルーレート）を引き上げた際に、これまで実際には
        到達していなかった未探索領域（+2.2〜+2.7rad付近）へ初めて到達し、
        過去最長となる1.867秒のLQIゲイン停止を引き起こすことが実験で
        判明した（詳細は研究ノート参照）。

        一方、今回の実験用途では「joint1の変形に必要な推力を得られれば
        よく、gimbal1を[-π/2, +π/2]の範囲内で動かせれば十分」という
        要件が明確になったため、πを跨がない3点移動方式に変更した:
          1) 現在角度に近い方の ±1.0 へ移動
          2) 0を通り抜けて符号反転した位置へ移動
          3) 最終角 -0.4 へ移動 → settle後にjoint1のコントローラを停止
        到達判定は self.fix_current（実測角度）ベースであり、旧方式の
        「コマンド積算カウンタと実角度のズレ」は構造的に発生しない。

[追加S] 上記3点移動の各区間の移動速度に、rev.11で導入した ramp ロジック
        （_next_spin_rate / _ramp_target_rate、fc_t_minに応じた連続速度
        補間）をそのまま適用した。到達判定が実測角度ベースになったこと
        で、rampによる速度変化が「見かけ上の到達判定のズレ」を生む心配
        がなくなり、安全にrampを組み込めるようになった。

[追加T] SPIN_RATE_FAST を 0.05 -> 0.025 rad/loop に変更（20Hzで
        1.0rad/s -> 0.5rad/s 相当）。C++側の fix_gimbal_slew_rate_ を
        0.5rad/sに設定したことに合わせ、Python側の最大コマンド速度も
        同じ上限に揃えた（どちらか一方だけ速くしても、遅い方が律速する
        だけで意味がないため）。

        既定の速度モードは fast/normal/slow ではなく "ramp" に変更した。
        fast/normal/slow は比較実験用にコードとしては残しているが、
        joint1_try1 の3点移動方式は常に _next_spin_rate() を通すため、
        --speed=ramp 以外を指定した場合は二値ヒステリシス方式で動く
        （rev.10までの挙動と同じ）。

--------------------------------------------------------------------------
rev.11 での変更（rev.10 からの差分）— 危険域スピン速度の連続化（"ramp"モード追加）
--------------------------------------------------------------------------
[追加Q] 危険域スピード切替を「二値のヒステリシス」から「fc_t_minに基づく
        連続補間」に変更した"ramp"モードを追加。

        (Q-1) RAMP_FC_HIGH（この値以上なら通常速度）から
              RAMP_FC_LOW（この値以下なら最大加速）まで、
              fc_t_minに対して線形補間で速度を決める。
        (Q-2) 速度の変化量自体にもスルーレート制限（RAMP_RATE_SLEW）を
              かけ、二重に滑らかにする。
        (Q-3) frac_linear を平方根カーブにし、fc_t_minがまだHIGHに近い
              段階から早めにFAST側へ立ち上げる。
        (Q-4) --speed=ramp で選択可能。fast/normal/slowは比較実験用
              にそのまま残す。

--------------------------------------------------------------------------
継続している変更（rev.10 まで）
--------------------------------------------------------------------------
[追加P] 危険域でのgimbal1スピン速度をfast/normal/slowの3モードから
        コマンドライン引数 --speed= で選べるようにした（比較実験用）。
[追加O] plan_debug に実際のgimbal角(opt_gimbal_angles_)が付加。
[追加N] fc_t_min に応じてgimbal1スピン速度を落とす（ヒステリシス付き）。
[追加M] /hydrus_xi/plan_debug の購読・ログ統合。
[追加K] psi1 スルー中の tau_min ガードと分枝リトライ。
[追加J] 特異点通過の事前準備 PREP。
[変更I] 変形順序 joint1 -> joint2,3。
[変更D/追加E/修正F] psi1 解放と GIMBAL_RELEASE。
[修正A/追加C] 固定完了判定、sweep モード。

--------------------------------------------------------------------------
既知の残課題（rev.14 時点）
--------------------------------------------------------------------------
残課題A: joint3符号反転時のjoint2,3畳み直しでの特異点（未対策）。
残課題D: sweepモードは現状動作しない（原因未調査、当面使用しない）。
残課題E: 3点移動方式が[-π/2, +π/2]の範囲内で全joint1角度に対して
         十分な安全マージンを保っているか、より広いjoint1角度レンジで
         の継続検証が必要。

使用例:
  rosrun hydrus_xi hydrus_xi_deformation_sequence.py 1.45 1 1
  rosrun hydrus_xi hydrus_xi_deformation_sequence.py 1.45 1 1 --speed=ramp
  rosrun hydrus_xi hydrus_xi_deformation_sequence.py 0.3 1 1 --speed=fast
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum
from controller_manager_msgs.srv import SwitchController


class Step(Enum):
    INIT              = 0
    PREP_JOINT23      = 1
    PREP_STABILIZE    = 2
    GIMBAL_FIX        = 3
    GIMBAL_STABILIZE  = 4
    JOINT1_SERVO      = 5
    JOINT1_STABILIZE  = 6
    GIMBAL_RELEASE    = 7
    JOINT23_SERVO     = 8
    JOINT23_STABILIZE = 9
    COMPLETE          = 10
    SWEEP             = 11
    JOINT1_TRY1       = 12


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.3
GIMBAL1_SIGN = +1.0

GIMBAL1_FORCE_BRANCH = None

# ---- 特異点通過の事前準備 ---------------------------------------------------
DANGER      = 0.25
PREP_ANGLE  = 1.2
# ---------------------------------------------------------------------------

# ---- rev.6 [追加K]: スルー中の tau_min ガードと分枝リトライ ------------------
SLEW_FC_T_MIN_MIN = 1.0
SLEW_GUARD_MIN_TRAVEL = 0.15
# ---------------------------------------------------------------------------

# ---- rev.7 [追加M]: plan_debug ジャンプ即時警告のしきい値 --------------------
JUMP_WARN_THRESH = 0.3
# ---------------------------------------------------------------------------

# ---- rev.10 [追加P]: fast/normal/slow の危険域固定速度（比較実験用） --------
SPIN_RATE_NORMAL = 0.02    # [rad/loop] 基準速度（20Hzで0.4rad/s）
SPIN_RATE_SLOW    = 0.005  # [rad/loop] slowモード時、危険域で使う速度
# ★rev.14 [追加T]: 0.05 (=1.0rad/s) -> 0.025 (=0.5rad/s) に変更。
#   C++側 fix_gimbal_slew_rate_ = 0.5rad/s と揃えた（片方だけ速くても
#   意味がないため）。rampモードの上限速度としても使われる。
SPIN_RATE_FAST    = 0.025  # [rad/loop] fastモード時、危険域で使う速度（rampの上限にも使う）

SPIN_SLOWDOWN_ENTER = 2.0  # [Nm] fast/slow/normalモード用の閾値（ヒステリシス）
SPIN_SLOWDOWN_EXIT  = 3.0
# ---------------------------------------------------------------------------

# ---- rev.11 [追加Q]: "ramp"モード（連続補間）用パラメータ -------------------
RAMP_FC_HIGH = 4.25    # [Nm] この値以上のfc_t_minでは通常速度(SPIN_RATE_NORMAL)
RAMP_FC_LOW  = 0.75    # [Nm] この値以下のfc_t_minでは最大速度(SPIN_RATE_FAST)
                       #      HIGHとLOWの間は線形補間する。
RAMP_RATE_SLEW = 0.003  # [rad/loop] 1ループあたりの速度自体の最大変化量。
                         #      fc_t_minが谷の底で細かく振動しても、
                         #      rate自体は滑らかにしか動かないようにする。

# ★rev.14 [追加T]: 既定を "ramp" に変更（3点移動方式が前提のため）
DEFAULT_DANGER_SPEED_MODE = "ramp"  # "normal"/"slow"/"fast"/"ramp"
# ---------------------------------------------------------------------------

GIMBAL_ERR_THRESH = 0.02
FC_T_MIN_REQUIRED = 0.01

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

# ---- rev.14 [追加R]: 3点移動方式のルート定義（joint1_try1で使用） -----------
TRY1_TARGET_A = -1.0
TRY1_TARGET_B = 1.0
TRY1_FINAL_TARGET = -0.4
TRY1_REACH_THRESH = 0.05
# ---------------------------------------------------------------------------

# ---- rev.15 [追加U]: joint2,3変形中の特異点通過ガード（gimbal1のrampと同じ原則） ----
#   q=(0.5,-1,0.5)実験でのフォースランディングを受けて追加。
#   固定レート(JOINT_RAMP_RATE)のまま特異条件（S1/S2）近傍を通過すると
#   危険域に長く留まることになるため、gimbal1のrampロジックと同じ
#   「fc_t_minが低いほど速く通り抜ける」原則を joint2,3 にも適用する。
JOINT23_RATE_NORMAL   = JOINT_RAMP_RATE        # 危険域外での既定速度（従来と同じ）
JOINT23_RATE_FAST     = JOINT_RAMP_RATE * 2.5  # 危険域での最大速度（gimbal1のfast倍率2.5を踏襲）
JOINT23_RAMP_FC_HIGH  = RAMP_FC_HIGH           # この値以上のfc_t_minでは通常速度
JOINT23_RAMP_FC_LOW   = RAMP_FC_LOW            # この値以下のfc_t_minでは最大速度
JOINT23_RAMP_RATE_SLEW = JOINT_RAMP_RATE * 0.2 # 1ループあたりの速度自体の最大変化量
# ---------------------------------------------------------------------------
JOINT23_SERVO_TIMEOUT = 15.0  #  [s] joint2,3変形が動かなくなった場合の強制打ち切り

class GimbalFixedSequencer(object):

    def __init__(self, q1, q2, q3, sweep=False, danger_speed_mode=DEFAULT_DANGER_SPEED_MODE):
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

        # rev.6 [追加K]: 分枝リトライ用
        self.branch_cand = {}
        self.branch_tried = []
        self.branch_current = None
        self.slew_start_psi = None

        # rev.10 [追加P]: fast/normal/slow用の危険域フラグ
        self.danger_speed_mode = danger_speed_mode if danger_speed_mode in ("fast", "normal", "slow", "ramp") else "normal"
        self.spin_slow_mode = False  # fast/normal/slowモードでのみ使うヒステリシス状態

        # rev.11 [追加Q]: rampモード用の現在速度（スルーレート制限の起点）
        self._ramp_current_rate = SPIN_RATE_NORMAL

        # rev.15 [追加U]: joint2,3変形用の独立したramp状態
        self._joint23_ramp_current_rate = JOINT23_RATE_NORMAL

        rospy.loginfo("[Seq] danger-zone spin speed mode = '%s' "
                      "(normal=%.4f, slow=%.4f, fast=%.4f rad/loop, "
                      "ramp: high=%.2f low=%.2fNm slew=%.4f)",
                      self.danger_speed_mode, SPIN_RATE_NORMAL, SPIN_RATE_SLOW, SPIN_RATE_FAST,
                      RAMP_FC_HIGH, RAMP_FC_LOW, RAMP_RATE_SLEW)

        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.fix_cmd_pub     = rospy.Publisher('/hydrus_xi/fixed_gimbal_cmd', Float64MultiArray, queue_size=1)

        self.switch_ctrl = rospy.ServiceProxy(
            '/hydrus_xi/controller_manager/switch_controller', SwitchController)

        self.plan_debug = {'stab_ok': 1.0, 'delta': 0.0, 'invalid': 0.0, 'jump': 0.0, 'gimbals': []}
        rospy.Subscriber('/hydrus_xi/plan_debug', Float64MultiArray, self._plan_debug_cb)

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

    def _plan_debug_cb(self, msg):
        if len(msg.data) < 4:
            return
        self.plan_debug['stab_ok'] = msg.data[0]
        self.plan_debug['delta']   = msg.data[1]
        self.plan_debug['invalid'] = msg.data[2]
        self.plan_debug['jump']    = msg.data[3]
        self.plan_debug['gimbals'] = list(msg.data[4:]) if len(msg.data) > 4 else []

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
            # rev.15 [追加U]: joint2,3のramp状態を、このステップに入る
            # たびに安全速度から再スタートさせる
            self._joint23_ramp_current_rate = JOINT23_RATE_NORMAL

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

    def _ramp(self, joints, rate=JOINT_RAMP_RATE):
        for j in joints:
            d = self._norm(self.target_q[j] - self.joint_targets[j])
            if abs(d) > rate:
                self.joint_targets[j] += math.copysign(rate, d)
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

    def _compute_branches(self, sign):
        a = self._norm(-GIMBAL1_MAG * GIMBAL1_SIGN * sign)
        b = self._norm(math.pi - a)
        return {'a': a, 'b': b}

    def _order_branches(self, cand):
        if GIMBAL1_FORCE_BRANCH in ('a', 'b'):
            return [GIMBAL1_FORCE_BRANCH]
        psi_now = self.fix_current
        da = abs(self._norm(cand['a'] - psi_now))
        db = abs(self._norm(cand['b'] - psi_now))
        return ['a', 'b'] if da <= db else ['b', 'a']

    def _check_fc_t_min(self):
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

    # rev.11 [追加Q]: rampモード用の連続速度補間
    def _ramp_target_rate(self, fc_t_min):
        """fc_t_minからramp目標速度を計算する（連続関数、閾値なし）。
        frac_linearを平方根カーブにすることで、fc_t_minがまだHIGHに近い
        （＝まだ十分安全な）段階からtarget_rateを早めに立ち上げ、
        谷本体に入る前にFAST側への「助走」を終わらせやすくする。
        """
        if fc_t_min >= RAMP_FC_HIGH:
            return SPIN_RATE_NORMAL
        if fc_t_min <= RAMP_FC_LOW:
            return SPIN_RATE_FAST
        frac_linear = (RAMP_FC_HIGH - fc_t_min) / (RAMP_FC_HIGH - RAMP_FC_LOW)  # 0(HIGH)->1(LOW)
        frac_linear = max(0.0, min(1.0, frac_linear))
        frac = frac_linear ** 0.5   # 平方根で早期に立ち上がる凹カーブ
        return SPIN_RATE_NORMAL + frac * (SPIN_RATE_FAST - SPIN_RATE_NORMAL)

    def _next_spin_rate(self):
        """このループで使うgimbal1移動速度を、現在のdanger_speed_modeに応じて決める。
        rev.14時点では joint1_try1 の3点移動の各区間でのみ使用される
        （旧・全周スピンロジックは廃止済み）。
        """
        if self.danger_speed_mode == "ramp":
            # rev.11: fc_t_minに基づく連続補間 + 速度自体へのスルーレート制限
            target_rate = self._ramp_target_rate(self.fc_t_min)
            step = target_rate - self._ramp_current_rate
            if abs(step) > RAMP_RATE_SLEW:
                step = math.copysign(RAMP_RATE_SLEW, step)
            self._ramp_current_rate += step
            return self._ramp_current_rate

        # rev.10までの fast/normal/slow: 二値ヒステリシス方式
        if self.spin_slow_mode:
            if self.fc_t_min > SPIN_SLOWDOWN_EXIT:
                self.spin_slow_mode = False
                rospy.loginfo("[Seq] spin: DANGER ZONE exited (mode='%s', fc_t_min=%.3f > %.2f)",
                              self.danger_speed_mode, self.fc_t_min, SPIN_SLOWDOWN_EXIT)
        else:
            if self.fc_t_min < SPIN_SLOWDOWN_ENTER:
                self.spin_slow_mode = True
                rospy.loginfo("[Seq] spin: DANGER ZONE entered (mode='%s', fc_t_min=%.3f < %.2f)",
                              self.danger_speed_mode, self.fc_t_min, SPIN_SLOWDOWN_ENTER)

        if not self.spin_slow_mode:
            return SPIN_RATE_NORMAL
        elif self.danger_speed_mode == "slow":
            return SPIN_RATE_SLOW
        elif self.danger_speed_mode == "fast":
            return SPIN_RATE_FAST
        else:  # "normal"
            return SPIN_RATE_NORMAL

    # rev.15 [追加U]: joint2,3変形用のramp速度計算（gimbal1の_ramp_target_rateと同じ考え方）
    def _joint23_ramp_target_rate(self, fc_t_min):
        """fc_t_minからjoint2,3の目標変化速度を計算する（連続関数、閾値なし）。
        gimbal1の_ramp_target_rateと同じ平方根カーブを使い、危険域に近づく
        ほど早期に速度を引き上げる。
        """
        if fc_t_min >= JOINT23_RAMP_FC_HIGH:
            return JOINT23_RATE_NORMAL
        if fc_t_min <= JOINT23_RAMP_FC_LOW:
            return JOINT23_RATE_FAST
        frac_linear = (JOINT23_RAMP_FC_HIGH - fc_t_min) / (JOINT23_RAMP_FC_HIGH - JOINT23_RAMP_FC_LOW)
        frac_linear = max(0.0, min(1.0, frac_linear))
        frac = frac_linear ** 0.5
        return JOINT23_RATE_NORMAL + frac * (JOINT23_RATE_FAST - JOINT23_RATE_NORMAL)

    def _next_joint23_rate(self):
        """このループで使うjoint2,3の変化速度を、fc_t_minに応じて連続的に決める。
        gimbal1のrampとは独立した状態（_joint23_ramp_current_rate）を持つ。
        """
        target_rate = self._joint23_ramp_target_rate(self.fc_t_min)
        step = target_rate - self._joint23_ramp_current_rate
        if abs(step) > JOINT23_RAMP_RATE_SLEW:
            step = math.copysign(JOINT23_RAMP_RATE_SLEW, step)
        self._joint23_ramp_current_rate += step
        return self._joint23_ramp_current_rate

    # ---------------- steps ----------------

    def _step_init(self):
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
        self._release_fix()
        done = self._ramp_to(['joint2', 'joint3'], [PREP_ANGLE, PREP_ANGLE])
        self._send_joint_cmd()

        rospy.loginfo_throttle(0.5, "[Seq] PREP: joint2,3 -> %.2f | q=(%.4f, %.4f) fc_t_min=%.3f",
                               PREP_ANGLE, self.current_q['joint2'], self.current_q['joint3'], self.fc_t_min)

        if done:
            rospy.loginfo("[Seq] PREP joint2,3 expanded -> stabilize")
            self._goto(Step.PREP_STABILIZE)

    def _step_prep_stabilize(self):
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] PREP settled (fc_t_min=%.3f) -> gimbal fix", self.fc_t_min)
            self._enter_gimbal_fix(self._diff('joint1'))

    def _enter_gimbal_fix(self, d1):
        sign = 1.0 if d1 >= 0.0 else -1.0
        self.branch_cand = self._compute_branches(sign)
        self.branch_tried = []
        order = self._order_branches(self.branch_cand)
        self._start_branch(order[0], d1)

    def _start_branch(self, key, d1=None):
        self.branch_current = key
        self.branch_tried.append(key)
        self.gimbal1_cmd = self.branch_cand[key]
        self.slew_start_psi = self.fix_current
        rospy.loginfo("[Seq] branch '%s': gimbal1 target = %+.3f rad (a=%+.3f b=%+.3f, psi_now=%+.3f)",
                      key, self.gimbal1_cmd, self.branch_cand['a'], self.branch_cand['b'], self.fix_current)
        self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        self._send_joint_cmd()
        self._hold_fix()

        traveled = abs(self._norm(self.fix_current - (self.slew_start_psi or self.fix_current)))

        if (self.fix_active and self.fix_enabled and self.fix_state_received
                and traveled > SLEW_GUARD_MIN_TRAVEL
                and self.fc_t_min < SLEW_FC_T_MIN_MIN):
            rospy.logwarn("[Seq] branch '%s' slew hits low tau_min=%.3f at psi1=%+.3f "
                          "(< %.2f). this path crosses singularity.",
                          self.branch_current, self.fc_t_min, self.fix_current, SLEW_FC_T_MIN_MIN)
            remaining = [k for k in ('a', 'b') if k not in self.branch_tried]
            if remaining:
                rospy.loginfo("[Seq] retrying with the other branch '%s'", remaining[0])
                self._release_fix()
                self._start_branch(remaining[0])
                return
            else:
                rospy.logerr("[Seq] ABORT: both branches cross singularity during slew "
                             "(q1_start=%+.3f). gimbal-fixed joint1 deform is infeasible "
                             "from this near-singular start.", self.current_q['joint1'])
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
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] stabilized (fc_t_min=%.3f) -> joint1 servo deform", self.fc_t_min)
            self._goto(Step.JOINT1_SERVO)

    def _step_joint1_servo(self):
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
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(['joint1']):
            rospy.loginfo("[Seq] joint1 settled (fc_t_min=%.3f) -> release psi1", self.fc_t_min)
            self._release_fix()
            self._goto(Step.GIMBAL_RELEASE)

    def _step_gimbal_release(self):
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
        self._release_fix()

        rate = self._next_joint23_rate()
        self._ramp(['joint2', 'joint3'], rate=rate)
        self._send_joint_cmd()

        rospy.loginfo_throttle(0.5, "[Seq] joint2,3: q=(%.4f, %.4f) tgt=(%.4f, %.4f) psi1=%+.3f "
                            "fc_t_min=%.3f rate=%.4f | stab_ok=%d invalid=%d jump=%.3f",
                            self.current_q['joint2'], self.current_q['joint3'],
                            self.target_q['joint2'], self.target_q['joint3'],
                            self.fix_current, self.fc_t_min, rate,
                            int(self.plan_debug['stab_ok']), int(self.plan_debug['invalid']),
                            self.plan_debug['jump'])

        if self.plan_debug['jump'] > JUMP_WARN_THRESH:
            rospy.logwarn(...)  # 既存のまま

        if self._reached(['joint2', 'joint3']):
            rospy.loginfo("[Seq] joint2,3 deform done -> stabilize")
            self._goto(Step.JOINT23_STABILIZE)
        elif self._elapsed() >= JOINT23_SERVO_TIMEOUT:
            rospy.logwarn("[Seq] joint2,3 servo timeout (%.1fs): q=(%.4f,%.4f) tgt=(%.4f,%.4f) "
                        "fc_t_min=%.3f. proceeding to stabilize anyway.",
                        JOINT23_SERVO_TIMEOUT, self.current_q['joint2'], self.current_q['joint3'],
                        self.target_q['joint2'], self.target_q['joint3'], self.fc_t_min)
            self._goto(Step.JOINT23_STABILIZE)

    def _step_joint23_stabilize(self):
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(['joint2', 'joint3']):
            rospy.loginfo("[Seq] all settled")
            self._goto(Step.JOINT1_TRY1)

    def _step_joint1_try1(self):
        """
        ★rev.14: 全周スピンによる診断を廃止し、3点経由の安全ルートを採用。
        各区間の移動は ramp ロジック（fc_t_minに応じた連続速度補間、
        rev.11由来）で制御し、真の谷に近づいたときだけ自動的に減速する。
        到達判定は実測角度(self.fix_current)ベースなので、旧スピン方式で
        問題になった「コマンド積算カウンタと実角度のズレ」は構造的に発生
        しない。

        ルート: 現在角度に近い方の ±1.0 -> 符号反転した位置 -> 最終角(-0.4)
        最大速度は SPIN_RATE_FAST（既定0.5rad/s相当）でキャップされる。
        """
        if not hasattr(self, '_try1_phase'):
            da = abs(self._norm(TRY1_TARGET_A - self.fix_current))
            db = abs(self._norm(TRY1_TARGET_B - self.fix_current))
            first_target = TRY1_TARGET_A if da <= db else TRY1_TARGET_B
            second_target = -first_target  # 0を通り抜けて線対称な位置へ

            self._try1_targets = [first_target, second_target, TRY1_FINAL_TARGET]
            self._try1_phase = 0
            self._try1_cmd = self.fix_current  # コマンドの起点は現在の実角度
            self.spin_slow_mode = False
            self._ramp_current_rate = SPIN_RATE_NORMAL

            rospy.loginfo("[Seq] joint1_try1: gimbal1 route = %+.3f -> %+.3f -> %+.3f "
                          "(joint1 still held, mode='%s', max=%.4f rad/loop)",
                          self._try1_targets[0], self._try1_targets[1], self._try1_targets[2],
                          self.danger_speed_mode, SPIN_RATE_FAST)

        if self._try1_phase < len(self._try1_targets):
            target = self._try1_targets[self._try1_phase]

            current_rate = self._next_spin_rate()  # fc_t_minに応じた速度（ramp/fast/normal/slow）

            err = self._norm(target - self._try1_cmd)
            if abs(err) <= current_rate:
                self._try1_cmd = target
            else:
                self._try1_cmd = self._norm(self._try1_cmd + math.copysign(current_rate, err))

            self.gimbal1_cmd = self._try1_cmd
            self._hold_fix()

            rospy.loginfo_throttle(
                0.25,
                "[Seq] joint1_try1: phase %d/%d cmd=%+.3f actual=%+.3f tgt=%+.3f "
                "fc_t_min=%.3f rate=%.4f | stab_ok=%d delta=%.2f invalid=%d jump=%.3f",
                self._try1_phase + 1, len(self._try1_targets),
                self._try1_cmd, self.fix_current, target, self.fc_t_min, current_rate,
                int(self.plan_debug['stab_ok']), self.plan_debug['delta'],
                int(self.plan_debug['invalid']), self.plan_debug['jump'])

            if self.plan_debug['jump'] > JUMP_WARN_THRESH:
                rospy.logwarn(
                    "[Seq] joint1_try1: JUMP EVENT phase=%d cmd=%+.3f actual=%+.3f fc_t_min=%.3f "
                    "jump=%.3f invalid=%d stab_ok=%d delta=%.2f rate=%.4f",
                    self._try1_phase + 1, self._try1_cmd, self.fix_current, self.fc_t_min,
                    self.plan_debug['jump'], int(self.plan_debug['invalid']),
                    int(self.plan_debug['stab_ok']), self.plan_debug['delta'], current_rate)

            # ★到達判定は実測角度ベース（旧スピン方式のtravelカウンタは使わない）
            if abs(self._norm(self.fix_current - target)) > TRY1_REACH_THRESH:
                return

            rospy.loginfo("[Seq] joint1_try1: phase %d/%d reached (%+.3f rad)",
                          self._try1_phase + 1, len(self._try1_targets), self.fix_current)
            self._try1_phase += 1
            return

        # ---- 最終目標(-0.4)に到達済み。settleしてcontroller1を停止 ----
        if not getattr(self, '_try1_reached_t', None):
            self._try1_reached_t = rospy.Time.now()
            rospy.loginfo("[Seq] joint1_try1: gimbal1 reached final %+.3f, waiting to settle", self.fix_current)

        settled = all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in self.joint_names)
        waited = (rospy.Time.now() - self._try1_reached_t).to_sec()
        if not settled and waited < 6.0:
            rospy.loginfo_throttle(0.5, "[Seq] joint1_try1: settling... (%.1fs)", waited)
            return

        if not getattr(self, '_try1_stopped', False):
            self.switch_ctrl(start_controllers=[],
                             stop_controllers=[JOINT1_CONTROLLER],
                             strictness=1)
            rospy.loginfo("[Seq] joint1_try1: settled, controller1 stopped")
            self._try1_stopped = True

    def _step_complete(self):
        self._send_joint_cmd()
        self._release_fix()

        if self._elapsed() < 0.1:
            tag = "ABORTED" if self.aborted else "DONE"
            rospy.loginfo("[Seq] %s%s  q=(%.4f/%.4f, %.4f/%.4f, %.4f/%.4f)  psi1=%+.3f  fc_t_min=%.3f  mode='%s'",
                          tag, " (prep used)" if self.prep_used else "",
                          self.current_q['joint1'], self.target_q['joint1'],
                          self.current_q['joint2'], self.target_q['joint2'],
                          self.current_q['joint3'], self.target_q['joint3'],
                          self.fix_current, self.fc_t_min, self.danger_speed_mode)

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

    def _loop(self, _event):
        try:
            if rospy.Time.now().is_zero():
                return
            if self.step_t0 is None:
                self.step_t0 = rospy.Time.now()

            {
                Step.INIT:              self._step_init,
                Step.PREP_JOINT23:      self._step_prep_joint23,
                Step.PREP_STABILIZE:    self._step_prep_stabilize,
                Step.GIMBAL_FIX:        self._step_gimbal_fix,
                Step.GIMBAL_STABILIZE:  self._step_gimbal_stabilize,
                Step.JOINT1_SERVO:      self._step_joint1_servo,
                Step.JOINT1_STABILIZE:  self._step_joint1_stabilize,
                Step.GIMBAL_RELEASE:    self._step_gimbal_release,
                Step.JOINT23_SERVO:     self._step_joint23_servo,
                Step.JOINT23_STABILIZE: self._step_joint23_stabilize,
                Step.COMPLETE:          self._step_complete,
                Step.SWEEP:             self._step_sweep,
                Step.JOINT1_TRY1:       self._step_joint1_try1,
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
        # ★rev.14: joint1_try1 の3点移動用フェーズ状態もリセットする
        for attr in ('_try1_phase', '_try1_targets', '_try1_cmd',
                     '_try1_reached_t', '_try1_stopped'):
            if hasattr(self, attr):
                delattr(self, attr)
        self._goto(Step.INIT)
        rospy.loginfo("[Seq] new target: (%.3f, %.3f, %.3f)", q1, q2, q3)

    def shutdown(self):
        self.timer.shutdown()


def main():
    rospy.init_node('hydrus_xi_gimbal_fixed_sequencer')

    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    sweep = '--sweep' in args
    args = [a for a in args if a != '--sweep']

    danger_speed_mode = DEFAULT_DANGER_SPEED_MODE
    speed_args = [a for a in args if a.startswith('--speed=')]
    if speed_args:
        candidate = speed_args[-1].split('=', 1)[1].strip().lower()
        if candidate in ("fast", "normal", "slow", "ramp"):
            danger_speed_mode = candidate
        else:
            rospy.logwarn("[Seq] unknown --speed='%s' (must be fast/normal/slow/ramp). "
                          "falling back to '%s'.", candidate, DEFAULT_DANGER_SPEED_MODE)
    args = [a for a in args if not a.startswith('--speed=')]

    q = [0.0, 0.0, 0.0]
    if len(args) >= 3:
        q = [float(args[i]) for i in range(3)]

    seq = GimbalFixedSequencer(q[0], q[1], q[2], sweep=sweep, danger_speed_mode=danger_speed_mode)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if seq.step == Step.COMPLETE:
            if sweep:
                rospy.loginfo("[Sweep] finished.")
                break
            print("\n" + "=" * 56)
            print(" Hydrus-Xi : joint1 thrust-deform (danger-zone speed mode='%s')" % seq.danger_speed_mode)
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
        else:
            rate.sleep()

    seq.shutdown()


if __name__ == '__main__':
    main()