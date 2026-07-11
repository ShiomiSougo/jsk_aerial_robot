#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）rev.3

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま joint1 の変形が成立するかを検証する。
  joint1 はサーボで駆動するため、失敗しても関節が暴走しない。

--------------------------------------------------------------------------
rev.3 での変更（rev.2 からの差分）
--------------------------------------------------------------------------
[変更D] joint1 の変形が終わったら psi_1 を最適化に返す。
        rev.2 は joint2,3 の変形中もシーケンス完了後も psi_1 を固定し続けていた。
        実測では固定によって tau_min が 5.03 -> 4.04 Nm と約 1.0 Nm 損している。
        joint1 が目標に着いた時点で凍らせておく理由はないので解放する。

[追加E] GIMBAL_RELEASE ステップ。
        解放直後、C++ 側は固定角からウォームスタートして
        +-gimbal_delta_angle_ (既定 0.2 rad/周期) で最適解へ歩いていく。
        例: -2.342 -> +3.19 なら 2.5 rad = 12 周期以上かかる。
        さらに stabilityCheck が落ちると delta_angle が pi にリセットされ、
        1 周期で大きく跳ぶ。

        したがって解放後は
          - C++ が fix_enabled=False を返した
          - psi_1 の変化率が十分小さくなった
          - tau_min が RELEASE_FC_T_MIN_OK 以上に回復した
          - joint の角速度が静止した
        の 4 条件が揃うまで joint2,3 の変形を始めない。

[修正F] 課題(8): `if self.fix_enabled:` で再送を判定していたため、
        C++ 側が一度でも False を返すと二度と固定に戻れなかった。
        Python 側の意図フラグ self.fix_active で判定し、
        fix_enabled は「C++ が受理したかの照合」にのみ使う。

--------------------------------------------------------------------------
rev.2 での修正（継続）
--------------------------------------------------------------------------
[修正A] 固定完了の判定に fix_enabled を必須条件として加える。

[修正B] 等価解の選択。生成モーメントは
            M1(psi1) = -A sin(psi1) + c
        であり sin(a) = sin(pi - a) なので psi1 = a と psi1 = pi - a は
        同じ joint1 モーメントを生む。現在角に近いほうを選ぶ。

        符号規約（本 rev で確定させたマッピング）:
            d1 = target_q1 - current_q1
            d1 < 0 (sign=-1): a = +0.3,  b = pi - 0.3
            d1 > 0 (sign=+1): a = -0.3,  b = 0.3 - pi   (= norm(pi - (-0.3)))
        a と b は sin が等しいので joint1 モーメントは同一。
        cos(a) と cos(pi - a) は符号が逆なので、直交する横力の向きは反転し、
        lambda_s / CoG フレーム / tau_min はすべて別物になる。
        GIMBAL1_FORCE_BRANCH で分枝を強制できるようにしてある。

[追加C] sweep モード。joint を一切動かさず psi1 を一周させ、
        (psi1, tau_min) を 20 Hz で流す。

            rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py --sweep
            rostopic echo -p /hydrus_xi/fixed_gimbal_state > sweep.csv

--------------------------------------------------------------------------
シーケンス（通常モード）
--------------------------------------------------------------------------
  (1) 開始・初期静定
  (2) 変形方向 sign(q1_target - q1_current) を受け取る
  (3) gimbal1 を固定角へスルーし、最適化変数から外す
  (4) 静定待ち
  (5) joint1 をサーボで変形（gimbal1 は固定のまま）
  (6) joint1 変形終了・静定待ち
  (7) gimbal1 を解放し、psi_1 が最適解へ収束するまで静定待ち   <- rev.3 で追加
  (8) joint2, 3 を従来どおりサーボ変形（psi_1 は自由）
  (9) 静定待ち・完了（gimbal1 は自由のまま）

使用例:
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py -0.6 0.9 0.9
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py --sweep
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum


class Step(Enum):
    INIT              = 0
    GIMBAL_FIX        = 1
    GIMBAL_STABILIZE  = 2
    JOINT1_SERVO      = 3
    JOINT1_STABILIZE  = 4
    GIMBAL_RELEASE    = 5      # rev.3 追加
    JOINT23_SERVO     = 6
    JOINT23_STABILIZE = 7
    COMPLETE          = 8
    SWEEP             = 9


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.3      # [rad] 固定するジンバル角の「大きさ」（正弦の引数として）
GIMBAL1_SIGN = +1.0     # 符号規約。実測後にマッピングが逆だったらここを反転させる

# 分枝の強制。None なら「現在角に近いほう」を自動選択（rev.2 の挙動）
#   'a' -> psi1 = a       (= -MAG * SIGN * sign)
#   'b' -> psi1 = pi - a
# 仮説検証（cos の符号が tau_min に効くか）のために使う。
GIMBAL1_FORCE_BRANCH = None

GIMBAL_ERR_THRESH = 0.02   # [rad] スルー完了判定
FC_T_MIN_REQUIRED = 0.01   # [Nm]  固定中これを下回ったら中断。実機では 1.5 ~ 2.0 にすること

ANGLE_ERROR_THRESHOLD = 0.03   # [rad] 関節到達判定
JOINT_RAMP_RATE = 0.0125       # [rad/loop] = 0.25 rad/s @ 20 Hz（論文の joint velocity）

STABILIZE_VEL_THRESH = 0.01    # [rad/s]
STABILIZE_HOLD_LOOPS = 20      # 連続してこの回数静止したら収束
STABILIZE_MIN_WAIT   = 1.0     # [s]
STABILIZE_TIMEOUT    = 6.0     # [s]

GIMBAL_SLEW_TIMEOUT = 20.0     # [s]

# ---- rev.3: gimbal1 解放後の静定 -------------------------------------------
# C++ 側は解放後、固定角から +-gimbal_delta_angle_ (既定 0.2 rad/周期) で歩く。
# psi1 の 1 ループあたりの変化がこれを下回ったら「歩き終わった」とみなす。
RELEASE_PSI_RATE_THRESH = 0.01   # [rad/loop] = 0.2 rad/s
RELEASE_HOLD_LOOPS      = 20     # 連続してこの回数静止したら収束（= 1.0 s）
RELEASE_MIN_WAIT        = 1.5    # [s] C++ が fix_enabled=False を返すまでの猶予込み
RELEASE_TIMEOUT         = 12.0   # [s] 2.5 rad / (0.2 rad/loop * 20 Hz) = 0.6 s なので十分
RELEASE_FC_T_MIN_OK     = 2.0    # [Nm] 解放後、これ以上に回復するまで待つ
# ---------------------------------------------------------------------------

# sweep モード
SWEEP_RATE = 0.02              # [rad/loop] = 0.4 rad/s（C++ 側 slew_rate 0.5 以下にすること）

LOOP_FREQ = 20.0
DT = 1.0 / LOOP_FREQ
# ---------------------------------------------------------------------------


class GimbalFixedSequencer(object):

    def __init__(self, q1, q2, q3, sweep=False):
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.target_q = {'joint1': q1, 'joint2': q2, 'joint3': q3}

        self.current_q  = {n: 0.0 for n in self.joint_names}
        self.current_dq = {n: 0.0 for n in self.joint_names}
        self.joint_targets = {n: 0.0 for n in self.joint_names}

        # C++ 側 fixed_gimbal_state のキャッシュ
        self.fix_enabled = False       # C++ が実際に固定モードに入っているか
        self.fix_target  = 0.0
        self.fix_current = 0.0         # 固定時: 固定角 / 自由時: 最適化された psi1
        self.fc_t_min    = 0.0
        self.fix_err     = math.pi
        self.fix_state_received = False

        # 【修正F】Python 側の意図。C++ の返り値ではなくこちらで再送を判定する
        self.fix_active = False

        self.sweep_mode  = sweep
        self.sweep_cmd   = 0.0
        self.sweep_travel = 0.0

        self.gimbal1_cmd = 0.0
        self.step = Step.INIT
        self.step_t0 = None
        self.hold_count = 0
        self.aborted = False

        # rev.3: 解放後の psi1 収束監視用
        self.release_hold = 0
        self.psi1_prev = None

        self.joints_ctrl_pub = rospy.Publisher('/hydrus_xi/joints_ctrl', JointState, queue_size=1)
        self.fix_cmd_pub     = rospy.Publisher('/hydrus_xi/fixed_gimbal_cmd', Float64MultiArray, queue_size=1)

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

    # 【修正F】意図フラグとセットで扱う
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

    def _reached(self, joints):
        return all(abs(self._diff(j)) <= ANGLE_ERROR_THRESHOLD for j in joints)

    # ---- 【修正B】等価解の選択（符号規約を本 rev で確定） ----
    def _pick_gimbal_target(self, sign):
        a = self._norm(-GIMBAL1_MAG * GIMBAL1_SIGN * sign)
        b = self._norm(math.pi - a)
        psi_now = self.fix_current

        if GIMBAL1_FORCE_BRANCH == 'a':
            best, why = a, "forced 'a'"
        elif GIMBAL1_FORCE_BRANCH == 'b':
            best, why = b, "forced 'b'"
        else:
            # cos(psi1) < 0 の枝（|psi1| が pi に近い側）が
            # feasible torque convex を保つことが実験で分かっている。
            # a, b は sin が等しく joint1 モーメントは同一なので、
            # cos の符号だけで選んでよい。
            best, why = (a, "cos<0") if math.cos(a) < 0 else (b, "cos<0")

        rospy.loginfo("[Seq] psi1 now=%+.3f | a=%+.3f (cos=%+.2f) / b=%+.3f (cos=%+.2f) -> pick %+.3f (%s)",
                    psi_now, a, math.cos(a), b, math.cos(b), best, why)
        return best


    def _check_fc_t_min(self):
        """
        ジンバル固定によって制御可能トルクが痩せすぎていないか監視。
        自由モード（psi1 解放中）では最適化が tau_min を最大化しているので監視しない。
        """
        if self.sweep_mode:
            return True
        # 【修正F】自分が固定を意図し、かつ C++ が受理しているときだけ監視
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
        """(1) 開始 / 初期ホバリング静定 -> (2) 変形方向"""
        self._send_joint_cmd()
        self._release_fix()

        if not self.fix_state_received:
            rospy.logwarn_throttle(2.0, "[Seq] waiting for fixed_gimbal_state ...")
            return

        if self._settled(self.joint_names):
            d1 = self._diff('joint1')
            if abs(d1) <= ANGLE_ERROR_THRESHOLD:
                rospy.loginfo("[Seq] joint1 already at target -> skip to joint2,3 (psi1 free)")
                self._goto(Step.JOINT23_SERVO)
                return

            sign = 1.0 if d1 >= 0.0 else -1.0
            self.gimbal1_cmd = self._pick_gimbal_target(sign)
            rospy.loginfo("[Seq] q1 diff = %+.4f -> gimbal1 target = %+.3f rad", d1, self.gimbal1_cmd)
            self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        """(3) gimbal1 を固定角へスルーさせ、最適化変数から外す"""
        self._send_joint_cmd()
        self._hold_fix()

        if not self._check_fc_t_min():
            return

        rospy.loginfo_throttle(0.5, "[Seq] slewing: en=%d cur=%+.3f tgt=%+.3f err=%.4f fc_t_min=%.3f",
                               self.fix_enabled, self.fix_current, self.fix_target,
                               self.fix_err, self.fc_t_min)

        # 【修正A】fix_enabled を必須条件に加える
        if self.fix_enabled and self.fix_err <= GIMBAL_ERR_THRESH:
            rospy.loginfo("[Seq] gimbal1 fixed at %+.3f rad (fc_t_min=%.3f Nm) -> stabilize",
                          self.fix_current, self.fc_t_min)
            self._goto(Step.GIMBAL_STABILIZE)
        elif self._elapsed() > GIMBAL_SLEW_TIMEOUT:
            rospy.logerr("[Seq] ABORT: gimbal1 slew timeout (en=%d err=%.4f)", self.fix_enabled, self.fix_err)
            self._release_fix()
            self.aborted = True
            self._goto(Step.COMPLETE)

    def _step_gimbal_stabilize(self):
        """(4) 固定後の静定"""
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] stabilized (fc_t_min=%.3f) -> joint1 servo deform", self.fc_t_min)
            self._goto(Step.JOINT1_SERVO)

    def _step_joint1_servo(self):
        """(5) joint1 をサーボで変形（gimbal1 は固定のまま）"""
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
        """(6) joint1 変形終了・静定 -> 【変更D】ここで psi1 を解放する"""
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        if self._settled(['joint1']):
            rospy.loginfo("[Seq] joint1 settled (fc_t_min=%.3f) -> release psi1", self.fc_t_min)
            self._release_fix()
            self._goto(Step.GIMBAL_RELEASE)

    # ---- 【追加E】gimbal1 解放後の静定 ----
    def _step_gimbal_release(self):
        """
        (7) psi1 を最適化に返し、最適解へ歩き終わるまで待つ。

        C++ 側は固定角からウォームスタートして +-gimbal_delta_angle_ / 周期で歩く。
        さらに stabilityCheck が落ちると delta_angle = pi にリセットされ大きく跳ぶ。
        joint2,3 を動かす前に、この過渡を完全に終わらせる。

        収束条件（すべて満たすこと）:
          - C++ が fix_enabled = False を返している
          - psi1 の 1 ループあたりの変化 < RELEASE_PSI_RATE_THRESH が連続 N 回
          - fc_t_min >= RELEASE_FC_T_MIN_OK
          - joint の角速度が静止
        """
        self._send_joint_cmd()
        self._release_fix()

        psi1 = self.fix_current
        if self.psi1_prev is None:
            self.psi1_prev = psi1
        d_psi1 = abs(self._norm(psi1 - self.psi1_prev))
        self.psi1_prev = psi1

        # C++ が解放を受理していなければ何も数えない
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
            rospy.loginfo("[Seq] psi1 released and settled at %+.3f rad (fc_t_min=%.3f Nm) -> joint2,3 deform",
                          psi1, self.fc_t_min)
            self._goto(Step.JOINT23_SERVO)
            return

        if self._elapsed() >= RELEASE_TIMEOUT:
            rospy.logwarn("[Seq] release settle timeout (%.1f s): psi1=%+.4f dpsi=%.4f fc_t_min=%.3f. "
                          "proceeding to joint2,3 anyway.",
                          RELEASE_TIMEOUT, psi1, d_psi1, self.fc_t_min)
            self._goto(Step.JOINT23_SERVO)

    def _step_joint23_servo(self):
        """(8) joint2,3 を従来どおり変形。psi1 は自由（最適化変数）"""
        self._release_fix()

        self._ramp(['joint2', 'joint3'])
        self._send_joint_cmd()

        rospy.loginfo_throttle(0.5, "[Seq] joint2,3: q=(%.4f, %.4f) tgt=(%.4f, %.4f) psi1=%+.3f fc_t_min=%.3f",
                               self.current_q['joint2'], self.current_q['joint3'],
                               self.target_q['joint2'], self.target_q['joint3'],
                               self.fix_current, self.fc_t_min)

        if self._reached(['joint2', 'joint3']):
            rospy.loginfo("[Seq] joint2,3 deform done -> stabilize")
            self._goto(Step.JOINT23_STABILIZE)

    def _step_joint23_stabilize(self):
        """(9) 最終静定"""
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(['joint2', 'joint3']):
            rospy.loginfo("[Seq] all settled")
            self._goto(Step.COMPLETE)

    def _step_complete(self):
        """完了。gimbal1 は自由のまま（rev.2 は固定を維持していた）"""
        self._send_joint_cmd()
        self._release_fix()

        if self._elapsed() < 0.1:
            tag = "ABORTED" if self.aborted else "DONE"
            rospy.loginfo("[Seq] %s  q=(%.4f/%.4f, %.4f/%.4f, %.4f/%.4f)  psi1=%+.3f  fc_t_min=%.3f", tag,
                          self.current_q['joint1'], self.target_q['joint1'],
                          self.current_q['joint2'], self.target_q['joint2'],
                          self.current_q['joint3'], self.target_q['joint3'],
                          self.fix_current, self.fc_t_min)

    # ---- 【追加C】sweep モード ----
    def _step_sweep(self):
        """
        joint を固定したまま psi1 を一周させ、(psi1, fc_t_min) を記録する。
        C++ 側の slew rate より遅く指令を動かすことで、指令に追従させる。
        """
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
                Step.INIT:              self._step_init,
                Step.GIMBAL_FIX:        self._step_gimbal_fix,
                Step.GIMBAL_STABILIZE:  self._step_gimbal_stabilize,
                Step.JOINT1_SERVO:      self._step_joint1_servo,
                Step.JOINT1_STABILIZE:  self._step_joint1_stabilize,
                Step.GIMBAL_RELEASE:    self._step_gimbal_release,
                Step.JOINT23_SERVO:     self._step_joint23_servo,
                Step.JOINT23_STABILIZE: self._step_joint23_stabilize,
                Step.COMPLETE:          self._step_complete,
                Step.SWEEP:             self._step_sweep,
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
            print(" Hydrus-Xi : gimbal1 fixed / joint1 servo deformation")
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