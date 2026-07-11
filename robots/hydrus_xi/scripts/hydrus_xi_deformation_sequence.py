#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）rev.5

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま joint1 の変形を成立させる。
  joint1 が line-shape 特異形態 (q1 ~ 0) を通過する場合は、事前に joint2,3 を
  展開して d（モーメントアーム）を稼いでおくことで、特異点を安全に通過する。

--------------------------------------------------------------------------
rev.5 での変更（rev.4 からの差分）
--------------------------------------------------------------------------
[変更I] 変形順序を joint1 -> joint2,3 に戻した（rev.3 と同じ本順序）。
        rev.4 で joint2,3 を先に畳む順序にしたところ、目標形態によっては
        joint2,3 が畳まれて d が短くなり、かえって joint1 の特異点通過が
        厳しくなった。順序を戻し、代わりに [追加J] の事前準備で対処する。

[追加J] 特異点通過の事前準備（PREP）。
        joint1 の始点 q1_start と目標 q1_target が作る区間が
        危険帯 [-DANGER, +DANGER]（既定 ±0.25 rad）と重なる場合のみ、
        joint1 を動かす前に joint2,3 を PREP_ANGLE（既定 0.5 rad）へ
        展開しておく。周囲リンクが開くと sin(beta)*lambda*d の d が伸び、
        joint1 が line-shape (q1~0) を通過する間も tau_min を支えられる。

        joint1 変形後、joint2,3 を PREP_ANGLE から最終目標角へ変形する。
        この joint2,3 変形は psi1 自由の 4 次元フル最適化なので安定。

        危険帯を通過しない変形（例: +1.57 -> +0.5）は準備を発動せず、
        従来どおり最短経路で動く。

--------------------------------------------------------------------------
継続している変更
--------------------------------------------------------------------------
[変更D] joint1 変形後に psi1 を解放（rev.3）。
[追加E] GIMBAL_RELEASE ステップ（rev.3）。
[修正F] 再送判定を self.fix_active で行う（rev.3）。
[変更H] _pick_gimbal_target は nearest（rev.4）。
[修正A] 固定完了判定に fix_enabled 必須（rev.2）。
[追加C] sweep モード（rev.2）。

--------------------------------------------------------------------------
シーケンス（通常モード, rev.5）
--------------------------------------------------------------------------
  (1) 開始・初期静定、危険帯通過を判定
  --- 危険帯を通過する場合のみ ---
  (P1) joint2,3 を PREP_ANGLE(0.5) へ展開（psi1 自由）
  (P2) 静定
  --- 共通 ---
  (2) gimbal1 を固定角へスルー
  (3) 固定後の静定
  (4) joint1 を変形（gimbal1 固定）
  (5) joint1 変形終了・静定
  (6) gimbal1 を解放し収束待ち
  (7) joint2,3 を最終目標角へ変形（psi1 自由）
  (8) 静定・完了

使用例:
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py -0.9 0.3 0.3
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
    PREP_JOINT23      = 1      # rev.5: 特異点通過の事前準備
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


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.3      # [rad] 固定するジンバル角の「大きさ」
GIMBAL1_SIGN = +1.0     # 符号規約。実測後にマッピングが逆だったらここを反転

# 分枝の強制。None なら nearest。'a'/'b' は検証用。
GIMBAL1_FORCE_BRANCH = None

# ---- rev.5: 特異点通過の事前準備 -------------------------------------------
DANGER      = 0.25     # [rad] 危険帯 [-DANGER, +DANGER]。joint1 がここを通ると準備発動
PREP_ANGLE  = 0.8      # [rad] 準備時に joint2,3 を展開する角度
# ---------------------------------------------------------------------------

GIMBAL_ERR_THRESH = 0.02   # [rad] スルー完了判定
FC_T_MIN_REQUIRED = 0.01   # [Nm]  固定中これを下回ったら中断。実機では 1.5 ~ 2.0 に

ANGLE_ERROR_THRESHOLD = 0.03   # [rad] 関節到達判定
JOINT_RAMP_RATE = 0.0125       # [rad/loop] = 0.25 rad/s @ 20 Hz

STABILIZE_VEL_THRESH = 0.01    # [rad/s]
STABILIZE_HOLD_LOOPS = 20      # 連続静止でこの回数
STABILIZE_MIN_WAIT   = 1.0     # [s]
STABILIZE_TIMEOUT    = 6.0     # [s]

GIMBAL_SLEW_TIMEOUT = 20.0     # [s]

# gimbal1 解放後の静定
RELEASE_PSI_RATE_THRESH = 0.01
RELEASE_HOLD_LOOPS      = 20
RELEASE_MIN_WAIT        = 1.5
RELEASE_TIMEOUT         = 12.0
RELEASE_FC_T_MIN_OK     = 2.0

# sweep モード
SWEEP_RATE = 0.02

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

        # rev.5: 準備を挟んだかどうか
        self.prep_used = False

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

    def _ramp_to(self, joints, goals):
        """joint_targets を任意の goal 値へランプ（準備用。target_q とは別の行き先）"""
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
        """joint1 の [start, target] 区間が危険帯 [-DANGER, +DANGER] と重なるか"""
        lo, hi = sorted([self.current_q['joint1'], self.target_q['joint1']])
        return (lo < DANGER) and (hi > -DANGER)

    def _pick_gimbal_target(self, sign):
        a = self._norm(-GIMBAL1_MAG * GIMBAL1_SIGN * sign)
        b = self._norm(math.pi - a)
        psi_now = self.fix_current

        da = abs(self._norm(a - psi_now))
        db = abs(self._norm(b - psi_now))

        if GIMBAL1_FORCE_BRANCH == 'a':
            best, why = a, "forced 'a'"
        elif GIMBAL1_FORCE_BRANCH == 'b':
            best, why = b, "forced 'b'"
        else:
            best, why = (a, "nearest") if da <= db else (b, "nearest")

        rospy.loginfo("[Seq] psi1 now=%+.3f | a=%+.3f (d=%.3f) / b=%+.3f (d=%.3f) -> pick %+.3f (%s)",
                      psi_now, a, da, b, db, best, why)
        return best

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
        sign = 1.0 if d1 >= 0.0 else -1.0
        self.gimbal1_cmd = self._pick_gimbal_target(sign)
        rospy.loginfo("[Seq] q1 diff = %+.4f -> gimbal1 target = %+.3f rad", d1, self.gimbal1_cmd)
        self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        """(2) gimbal1 を固定角へスルー"""
        self._send_joint_cmd()
        self._hold_fix()
        if not self._check_fc_t_min():
            return

        rospy.loginfo_throttle(0.5, "[Seq] slewing: en=%d cur=%+.3f tgt=%+.3f err=%.4f fc_t_min=%.3f",
                               self.fix_enabled, self.fix_current, self.fix_target,
                               self.fix_err, self.fc_t_min)

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
        """(7) joint2,3 を最終目標角へ変形（psi1 自由）
             準備で 0.5 に開いた場合はそこから target_q へ畳み直す。"""
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
        """(8) 最終静定"""
        self._send_joint_cmd()
        self._release_fix()

        if self._settled(['joint2', 'joint3']):
            rospy.loginfo("[Seq] all settled")
            self._goto(Step.COMPLETE)

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
            print(" Hydrus-Xi : joint1 thrust-deform with singularity prep")
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