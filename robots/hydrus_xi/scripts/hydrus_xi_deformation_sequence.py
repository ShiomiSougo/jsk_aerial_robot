#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）rev.2

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま飛行と変形が成立するかを検証する。
  joint1 はサーボで駆動するため、失敗しても関節が暴走しない。

--------------------------------------------------------------------------
rev.2 での修正
--------------------------------------------------------------------------
[修正A] 固定完了の判定が、固定が始まる前にすり抜けていた。
        C++ 側は無効時に err=0 を publish していたため（現在は err=pi）、
        Python 側も fix_enabled を必須条件に加える。

[修正B] 等価解の選択。生成モーメントは
            M1(psi1) = -A sin(psi1) + c
        であり sin(a) = sin(pi - a) なので、
            psi1 = a  と  psi1 = pi - a
        は同じモーメントを生む。現在角に近いほうを選べば、
        (1) スルー時間が短く、(2) tau_min の谷を通過しない。

        rev.1 では psi1 = 3.182 -> -0.500 (2.60 rad) と遠回りし、
        途中の psi1 ~ -1.18 rad で tau_min が 5.03 -> 0.94 Nm まで落ちて
        中断した。等価解 pi - (-0.5) = -2.642 なら移動量は 0.46 rad で済む。

[追加C] sweep モード。joint を一切動かさず psi1 を一周させ、
        (psi1, tau_min) を 20 Hz で標準出力へ流す。
        tau_min(psi1) 曲線を実測するための最短経路。

            rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py --sweep
            rostopic echo -p /hydrus_xi/fixed_gimbal_state > sweep.csv

--------------------------------------------------------------------------
シーケンス（通常モード）
--------------------------------------------------------------------------
  (1) 開始
  (2) 変形方向 sign(q1_target - q1_current) を受け取る
  (3) gimbal1 を等価解のうち近いほうへスルーし、最適化変数から外す
  (4) 静定待ち
  (5) joint1 をサーボで変形（joint2,3 と同じ位置ランプ）
  (6) joint1 変形終了・静定待ち
  (7) joint2, 3 を従来どおりサーボ変形

使用例:
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py 0.9 0.9 0.9
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
    JOINT23_SERVO     = 5
    JOINT23_STABILIZE = 6
    COMPLETE          = 7
    SWEEP             = 8


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.3      # [rad] 固定するジンバル角の「大きさ」（正弦の引数として）0.3くらいがよいか？
GIMBAL1_SIGN = +1.0     # 符号規約が未同定。実測後にここを反転させる

GIMBAL_ERR_THRESH = 0.02   # [rad] スルー完了判定
FC_T_MIN_REQUIRED = 0.01    # [Nm]  これを下回ったら中断（sweep モードでは無効）本当は1くらいが妥当

ANGLE_ERROR_THRESHOLD = 0.03   # [rad] 関節到達判定
JOINT_RAMP_RATE = 0.0125       # [rad/loop] = 0.25 rad/s @ 20 Hz（論文の joint velocity）

STABILIZE_VEL_THRESH = 0.01    # [rad/s]
STABILIZE_HOLD_LOOPS = 20      # 連続してこの回数静止したら収束
STABILIZE_MIN_WAIT   = 1.0     # [s]
STABILIZE_TIMEOUT    = 6.0     # [s]

GIMBAL_SLEW_TIMEOUT = 20.0     # [s]

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
        self.fix_enabled = False
        self.fix_target  = 0.0
        self.fix_current = 0.0
        self.fc_t_min    = 0.0
        self.fix_err     = math.pi
        self.fix_state_received = False

        self.sweep_mode  = sweep
        self.sweep_cmd   = 0.0
        self.sweep_travel = 0.0

        self.gimbal1_cmd = 0.0
        self.step = Step.INIT
        self.step_t0 = None
        self.hold_count = 0
        self.aborted = False

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
        self._send_fix_cmd(False, 0.0)

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

    def _goto(self, step):
        self.step = step
        self.step_t0 = rospy.Time.now()
        self.hold_count = 0

    def _elapsed(self):
        return (rospy.Time.now() - self.step_t0).to_sec()

    def _settled(self, joints):
        if self._elapsed() < STABILIZE_MIN_WAIT:
            return False
        if all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in joints):
            self.hold_count += 1
        else:
            self.hold_count = 0
        return self.hold_count >= STABILIZE_HOLD_LOOPS or self._elapsed() >= STABILIZE_TIMEOUT

    def _ramp(self, joints):
        for j in joints:
            d = self._norm(self.target_q[j] - self.joint_targets[j])
            if abs(d) > JOINT_RAMP_RATE:
                self.joint_targets[j] += math.copysign(JOINT_RAMP_RATE, d)
            else:
                self.joint_targets[j] = self.target_q[j]

    def _reached(self, joints):
        return all(abs(self._diff(j)) <= ANGLE_ERROR_THRESHOLD for j in joints)

    # ---- 【修正B】等価解の選択 ----
    def _pick_gimbal_target(self, sign):
        """
        M1(psi1) = -A sin(psi1) + c であり sin(a) = sin(pi - a) なので、
        psi1 = a と psi1 = pi - a は同じ内部モーメントを生む。
        現在角に近いほうを選ぶことで
          - スルー時間が短くなる
          - tau_min の谷（psi1 ~ +-pi/2 の反対枝）を通過しない
        """
        a = self._norm(GIMBAL1_MAG * GIMBAL1_SIGN * sign)
        b = self._norm(math.pi - a)
        psi_now = self.fix_current

        da = abs(self._norm(a - psi_now))
        db = abs(self._norm(b - psi_now))
        best = a if da <= db else b

        rospy.loginfo("[Seq] psi1 now=%+.3f | candidates %+.3f (d=%.3f) / %+.3f (d=%.3f) -> pick %+.3f",
                      psi_now, a, da, b, db, best)
        return best

    def _check_fc_t_min(self):
        """ジンバル固定によって制御可能トルクが痩せすぎていないか監視"""
        if self.sweep_mode:
            return True
        if not self.fix_enabled or not self.fix_state_received:
            return True
        if self.fc_t_min < FC_T_MIN_REQUIRED:
            rospy.logerr("[Seq] ABORT: fc_t_min = %.3f Nm < %.3f (psi1=%+.3f). gimbal fix released.",
                         self.fc_t_min, FC_T_MIN_REQUIRED, self.fix_current)
            self._send_fix_cmd(False, 0.0)
            self.aborted = True
            self._goto(Step.COMPLETE)
            return False
        return True

    # ---------------- steps ----------------

    def _step_init(self):
        """(1) 開始 / 初期ホバリング静定 -> (2) 変形方向"""
        self._send_joint_cmd()
        self._send_fix_cmd(False, 0.0)

        if not self.fix_state_received:
            rospy.logwarn_throttle(2.0, "[Seq] waiting for fixed_gimbal_state ...")
            return

        if self._settled(self.joint_names):
            d1 = self._diff('joint1')
            if abs(d1) <= ANGLE_ERROR_THRESHOLD:
                rospy.loginfo("[Seq] joint1 already at target -> skip to joint2,3")
                self._goto(Step.JOINT23_SERVO)
                return

            sign = 1.0 if d1 >= 0.0 else -1.0
            self.gimbal1_cmd = self._pick_gimbal_target(sign)
            rospy.loginfo("[Seq] q1 diff = %+.4f -> gimbal1 target = %+.3f rad", d1, self.gimbal1_cmd)
            self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        """(3) gimbal1 を固定角へスルーさせ、最適化変数から外す"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)

        if not self._check_fc_t_min():
            return

        rospy.loginfo_throttle(0.5, "[Seq] slewing: en=%d cur=%+.3f tgt=%+.3f err=%.4f fc_t_min=%.3f",
                               self.fix_enabled, self.fix_current, self.fix_target,
                               self.fix_err, self.fc_t_min)

        # 【修正A】fix_enabled を必須条件に加える（無効時 err=pi なのでこれ単独でも防げるが二重に）
        if self.fix_enabled and self.fix_err <= GIMBAL_ERR_THRESH:
            rospy.loginfo("[Seq] gimbal1 fixed at %+.3f rad (fc_t_min=%.3f Nm) -> stabilize",
                          self.fix_current, self.fc_t_min)
            self._goto(Step.GIMBAL_STABILIZE)
        elif self._elapsed() > GIMBAL_SLEW_TIMEOUT:
            rospy.logerr("[Seq] ABORT: gimbal1 slew timeout (en=%d err=%.4f)", self.fix_enabled, self.fix_err)
            self._send_fix_cmd(False, 0.0)
            self.aborted = True
            self._goto(Step.COMPLETE)

    def _step_gimbal_stabilize(self):
        """(4) 動作の安定化"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] stabilized (fc_t_min=%.3f) -> joint1 servo deform", self.fc_t_min)
            self._goto(Step.JOINT1_SERVO)

    def _step_joint1_servo(self):
        """(5) joint1 をサーボで変形（gimbal1 は固定のまま）"""
        self._send_fix_cmd(True, self.gimbal1_cmd)
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
        """(6) joint1 変形終了"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        if self._settled(['joint1']):
            rospy.loginfo("[Seq] joint1 settled -> joint2,3 servo deform")
            self._goto(Step.JOINT23_SERVO)

    def _step_joint23_servo(self):
        """(7) joint2,3 は従来どおり（gimbal1 は固定を維持）"""
        if self.fix_enabled:
            self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        self._ramp(['joint2', 'joint3'])
        self._send_joint_cmd()

        if self._reached(['joint2', 'joint3']):
            rospy.loginfo("[Seq] joint2,3 deform done -> stabilize")
            self._goto(Step.JOINT23_STABILIZE)

    def _step_joint23_stabilize(self):
        self._send_joint_cmd()
        if self.fix_enabled:
            self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        if self._settled(['joint2', 'joint3']):
            rospy.loginfo("[Seq] all settled")
            self._goto(Step.COMPLETE)

    def _step_complete(self):
        self._send_joint_cmd()
        if not self.aborted:
            self._send_fix_cmd(True, self.gimbal1_cmd)   # 固定は維持したまま終了
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
        self._send_fix_cmd(True, self.sweep_cmd)

        rospy.loginfo_throttle(0.25, "[Sweep] psi1=%+.4f  fc_t_min=%.4f  travel=%.2f/%.2f rad",
                               self.fix_current, self.fc_t_min, self.sweep_travel, 2 * math.pi)

        if self.sweep_travel >= 2 * math.pi:
            rospy.loginfo("[Sweep] one full revolution done. releasing gimbal fix.")
            self._send_fix_cmd(False, 0.0)
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
        self._send_fix_cmd(False, 0.0)     # 一旦解放して psi1 を再最適化させる

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