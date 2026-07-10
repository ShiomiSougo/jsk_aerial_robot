#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hydrus-Xi 変形シーケンス（gimbal1 固定・joint1 サーボ駆動版）

目的:
  psi_1 (gimbal1 の vectoring 角) を固定したまま飛行と変形が成立するかを検証する。
  joint1 はサーボで駆動するため、失敗しても関節が暴走しない。

シーケンス:
  ① 開始
  ② 変形方向 sign(q1_target - q1_current) を受け取る
  ③ gimbal1 を GIMBAL1_MAG * sign へスルーさせ、最適化からも外す
  ④ 静定待ち
  ⑤ joint1 をサーボで変形（joint2,3 と同じ位置ランプ）
  ⑥ joint1 変形終了・静定待ち
  ⑦ joint2, 3 を従来どおりサーボ変形

旧版から削除した機能:
  - controller_manager の switch_controller / list_controllers（joint1 を解放しないため不要）
  - target_internal_moment トピックとモーメント P 制御（psi_1 固定で不要）
  - プリロード段階、摩擦補償トルク、減速帯

使用例:
  rosrun hydrus_xi hydrus_xi_gimbal_fixed_sequence.py 1.57 1.57 1.57
"""

import rospy
import sys
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import Enum


class Step(Enum):
    INIT             = 0
    GIMBAL_FIX       = 1
    GIMBAL_STABILIZE = 2
    JOINT1_SERVO     = 3
    JOINT1_STABILIZE = 4
    JOINT23_SERVO    = 5
    JOINT23_STABILIZE = 6
    COMPLETE         = 7


# ---- 実験パラメータ ---------------------------------------------------------
GIMBAL1_MAG  = 0.5      # [rad] 固定するジンバル角の大きさ
GIMBAL1_SIGN = +1.0     # 符号規約が未同定なので、実測後にここを反転させる
GIMBAL_ERR_THRESH = 0.02   # [rad] スルー完了判定
FC_T_MIN_REQUIRED = 1.0    # [Nm]  この値を下回ったら中断（fc_t_min_thresh_ より少し緩め）

ANGLE_ERROR_THRESHOLD = 0.03   # [rad] 関節到達判定
JOINT_RAMP_RATE = 0.0125       # [rad/loop] = 0.25 rad/s @ 20 Hz（論文の joint velocity）

STABILIZE_VEL_THRESH   = 0.01  # [rad/s]
STABILIZE_HOLD_LOOPS   = 20    # 連続してこの回数静止したら収束
STABILIZE_MIN_WAIT     = 1.0   # [s]
STABILIZE_TIMEOUT      = 6.0   # [s]

GIMBAL_SLEW_TIMEOUT = 15.0     # [s]

LOOP_FREQ = 20.0
DT = 1.0 / LOOP_FREQ
# ---------------------------------------------------------------------------


class GimbalFixedSequencer(object):

    def __init__(self, q1, q2, q3):
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.target_q = {'joint1': q1, 'joint2': q2, 'joint3': q3}

        self.current_q  = {n: 0.0 for n in self.joint_names}
        self.current_dq = {n: 0.0 for n in self.joint_names}
        self.joint_targets = {n: 0.0 for n in self.joint_names}

        # C++ 側 fixed_gimbal_state のキャッシュ
        self.fix_enabled  = False
        self.fix_target   = 0.0
        self.fix_current  = 0.0
        self.fc_t_min     = 0.0
        self.fix_err      = math.pi
        self.fix_state_received = False

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

        for n in self.joint_names:
            self.joint_targets[n] = self.current_q[n]

        rate = rospy.Rate(10)
        while self.joints_ctrl_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rate.sleep()

        self._send_joint_cmd()
        self._send_fix_cmd(False, 0.0)

        rospy.loginfo("[Seq] init done. q=(%.3f, %.3f, %.3f) -> target=(%.3f, %.3f, %.3f)",
                      self.current_q['joint1'], self.current_q['joint2'], self.current_q['joint3'],
                      q1, q2, q3)

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
        """速度が閾値以下の状態が連続 STABILIZE_HOLD_LOOPS 回続いたら True"""
        if self._elapsed() < STABILIZE_MIN_WAIT:
            return False
        if all(abs(self.current_dq[j]) < STABILIZE_VEL_THRESH for j in joints):
            self.hold_count += 1
        else:
            self.hold_count = 0
        return self.hold_count >= STABILIZE_HOLD_LOOPS or self._elapsed() >= STABILIZE_TIMEOUT

    def _ramp(self, joints):
        """位置指令を JOINT_RAMP_RATE でランプさせる"""
        for j in joints:
            d = self._norm(self.target_q[j] - self.joint_targets[j])
            if abs(d) > JOINT_RAMP_RATE:
                self.joint_targets[j] += math.copysign(JOINT_RAMP_RATE, d)
            else:
                self.joint_targets[j] = self.target_q[j]

    def _reached(self, joints):
        return all(abs(self._diff(j)) <= ANGLE_ERROR_THRESHOLD for j in joints)

    def _check_fc_t_min(self):
        """ジンバル固定によって制御可能トルクが痩せすぎていないか監視"""
        if not self.fix_enabled or not self.fix_state_received:
            return True
        if self.fc_t_min < FC_T_MIN_REQUIRED:
            rospy.logerr("[Seq] ABORT: fc_t_min = %.3f Nm < %.3f. gimbal fix released.",
                         self.fc_t_min, FC_T_MIN_REQUIRED)
            self._send_fix_cmd(False, 0.0)
            self.aborted = True
            self._goto(Step.COMPLETE)
            return False
        return True

    # ---------------- steps ----------------

    def _step_init(self):
        """① 開始 / 初期ホバリング静定"""
        self._send_joint_cmd()
        if self._settled(self.joint_names):
            # ② 変形方向を受け取る
            d1 = self._diff('joint1')
            sign = 1.0 if d1 >= 0.0 else -1.0
            self.gimbal1_cmd = GIMBAL1_MAG * GIMBAL1_SIGN * sign

            rospy.loginfo("[Seq] q1 diff = %+.4f rad -> gimbal1 target = %+.3f rad", d1, self.gimbal1_cmd)
            self._goto(Step.GIMBAL_FIX)

    def _step_gimbal_fix(self):
        """③ gimbal1 を固定角へスルーさせ、最適化変数から外す"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)

        if not self.fix_state_received:
            if self._elapsed() > 3.0:
                rospy.logwarn_throttle(1.0, "[Seq] no fixed_gimbal_state. is the navigator running?")
            return

        if not self._check_fc_t_min():
            return

        rospy.loginfo_throttle(0.5, "[Seq] slewing gimbal1: cur=%+.3f tgt=%+.3f err=%.4f fc_t_min=%.3f",
                               self.fix_current, self.fix_target, self.fix_err, self.fc_t_min)

        if self.fix_err <= GIMBAL_ERR_THRESH:
            rospy.loginfo("[Seq] gimbal1 fixed at %+.3f rad (fc_t_min=%.3f Nm) -> stabilize",
                          self.fix_current, self.fc_t_min)
            self._goto(Step.GIMBAL_STABILIZE)
        elif self._elapsed() > GIMBAL_SLEW_TIMEOUT:
            rospy.logerr("[Seq] ABORT: gimbal1 slew timeout (err=%.4f)", self.fix_err)
            self._send_fix_cmd(False, 0.0)
            self.aborted = True
            self._goto(Step.COMPLETE)

    def _step_gimbal_stabilize(self):
        """④ 動作の安定化"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        if self._settled(self.joint_names):
            rospy.loginfo("[Seq] stabilized -> joint1 servo deform")
            self._goto(Step.JOINT1_SERVO)

    def _step_joint1_servo(self):
        """⑤ joint1 をサーボで変形（gimbal1 は固定のまま）"""
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
        """⑥ joint1 変形終了"""
        self._send_joint_cmd()
        self._send_fix_cmd(True, self.gimbal1_cmd)
        if not self._check_fc_t_min():
            return

        if self._settled(['joint1']):
            rospy.loginfo("[Seq] joint1 settled -> joint2,3 servo deform")
            self._goto(Step.JOINT23_SERVO)

    def _step_joint23_servo(self):
        """⑦ joint2,3 は従来どおり（gimbal1 は固定を維持）"""
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
            rospy.loginfo("[Seq] %s  q=(%.4f/%.4f, %.4f/%.4f, %.4f/%.4f)  fc_t_min=%.3f", tag,
                          self.current_q['joint1'], self.target_q['joint1'],
                          self.current_q['joint2'], self.target_q['joint2'],
                          self.current_q['joint3'], self.target_q['joint3'],
                          self.fc_t_min)

    # ---------------- main loop ----------------

    def _loop(self, _event):
        try:
            if rospy.Time.now().is_zero():
                return
            if self.step_t0 is None:
                self.step_t0 = rospy.Time.now()

            handler = {
                Step.INIT:              self._step_init,
                Step.GIMBAL_FIX:        self._step_gimbal_fix,
                Step.GIMBAL_STABILIZE:  self._step_gimbal_stabilize,
                Step.JOINT1_SERVO:      self._step_joint1_servo,
                Step.JOINT1_STABILIZE:  self._step_joint1_stabilize,
                Step.JOINT23_SERVO:     self._step_joint23_servo,
                Step.JOINT23_STABILIZE: self._step_joint23_stabilize,
                Step.COMPLETE:          self._step_complete,
            }[self.step]
            handler()
        except Exception as e:
            rospy.logerr("[Seq] loop error: %s", str(e))

    def new_target(self, q1, q2, q3):
        for n in self.joint_names:
            self.joint_targets[n] = self.current_q[n]
        self._send_joint_cmd()

        self.target_q = {'joint1': q1, 'joint2': q2, 'joint3': q3}
        self.aborted = False
        self._goto(Step.INIT)
        rospy.loginfo("[Seq] new target: (%.3f, %.3f, %.3f)", q1, q2, q3)

    def shutdown(self):
        self.timer.shutdown()


def main():
    rospy.init_node('hydrus_xi_gimbal_fixed_sequencer')

    q = [0.0, 0.0, 0.0]
    if len(sys.argv) >= 4:
        q = [float(sys.argv[i]) for i in (1, 2, 3)]

    seq = GimbalFixedSequencer(*q)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if seq.step == Step.COMPLETE:
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