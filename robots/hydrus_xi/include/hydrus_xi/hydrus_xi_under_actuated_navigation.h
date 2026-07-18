// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, JSK Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the JSK Lab nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <algorithm>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <nlopt.hpp>
#include <OsqpEigen/OsqpEigen.h>

/* ★ 固定ジンバル指令・状態フィードバック用 */
#include <std_msgs/Float64MultiArray.h>
#include <mutex>

namespace aerial_robot_navigation
{
  class HydrusXiUnderActuatedNavigator : public BaseNavigator
  {
  public:
    HydrusXiUnderActuatedNavigator();
    ~HydrusXiUnderActuatedNavigator();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    inline boost::shared_ptr<HydrusTiltedRobotModel> getRobotModelForPlan() { return robot_model_for_plan_;}
    inline OsqpEigen::Solver& getYawRangeLPSolver() { return yaw_range_lp_solver_;}

    inline KDL::JntArray& getJointPositionsForPlan()  {return joint_positions_for_plan_;}
    inline const double& getMaxMinYaw() const { return max_min_yaw_;}

    inline const double& getForceNormWeight() const { return force_norm_weight_;}
    inline const double& getForceVariantWeight() const { return force_variant_weight_;}
    inline const double& getYawTorqueWeight() const { return yaw_torque_weight_;}
    inline const double& getFCTMinWeight() const { return fc_t_min_weight_;}
    inline const double& getBaselinkRotThresh() const { return baselink_rot_thresh_;}
    inline const double& getFCTMinThresh() const { return fc_t_min_thresh_;}

    const std::vector<std::string>& getControlNames() const { return control_gimbal_names_; }
    const std::vector<int>& getControlIndices() const { return control_gimbal_indices_; }

    const bool getPlanVerbose() const { return plan_verbose_; }

    void setMaxMinYaw(const double max_min_yaw) { max_min_yaw_ = max_min_yaw;}

    /* ★ nlopt の自由変数 x を全ジンバル角ベクトルへ展開する。
     *   無名名前空間の applyGimbalAngles() から planner-> 経由で呼ぶため public。 */
    std::vector<double> composeGimbalAngles(const std::vector<double>& x);

  private:
    ros::Publisher gimbal_ctrl_pub_;
    std::thread plan_thread_;
    boost::shared_ptr<HydrusTiltedRobotModel> robot_model_for_plan_;
    OsqpEigen::Solver yaw_range_lp_solver_;
    boost::shared_ptr<nlopt::opt> vectoring_nl_solver_;  // 未使用（互換のため残置）

    KDL::JntArray joint_positions_for_plan_;
    std::vector<std::string> control_gimbal_names_;
    std::vector<int> control_gimbal_indices_;
    double max_min_yaw_;

    bool plan_verbose_;
    bool maximize_yaw_;
    double force_norm_weight_; // cost func
    double force_variant_weight_; // cost func
    double yaw_torque_weight_; // cost func
    double fc_t_min_weight_; // cost func
    double baselink_rot_thresh_; // constraint func
    double fc_t_min_thresh_; // constraint func
    double gimbal_delta_angle_; // configuration state

    std::vector<double> opt_gimbal_angles_, prev_opt_gimbal_angles_;

    void threadFunc();
    bool plan();

    void rosParamInit() override;

    /* ================= ★ 固定ジンバル関連（ここから） =================
     *
     * 指定した 1 つのジンバル（既定 "gimbal1"）を nlopt の最適化変数から
     * 外し、外部から与えた固定角として扱う。最適化次元は N -> N-1 に落ちる。
     * これにより内部モーメント M1 は静止推力 lambda_1 のスカラー倍として
     * 一意に決まり、ペナルティ項によるモーメント制御が不要になる。
     */
    ros::Subscriber fix_gimbal_cmd_sub_;    // ~/fixed_gimbal_cmd   [enable, angle]
    ros::Publisher  fix_gimbal_state_pub_;  // ~/fixed_gimbal_state [en, tgt, cur, fc_t_min, err]
    /* ★ [追加] nlopt探索範囲リセット・解の跳躍を診断するためのpublisher
     *   /hydrus_xi/plan_debug : Float64MultiArray
     *   [0] prev_stability_ok (1.0 = 前周期の解でstabilityCheck OK, 0.0 = NG -> delta_angle=PI)
     *   [1] delta_angle_used  [rad] このplan()周期で実際に使った探索半幅
     *   [2] invalid_cnt       このplan()周期のnlopt内でstabilityCheckが失敗した回数
     *   [3] max_gimbal_jump   [rad] gimbal2,3,4のうち前周期解との最大差分(固定ジンバルは除く)
     */
    ros::Publisher  plan_debug_pub_;
    int    last_invalid_cnt_;
    double last_max_jump_;
    
    boost::shared_ptr<nlopt::opt> vectoring_nl_solver_full_;     // N   次元
    boost::shared_ptr<nlopt::opt> vectoring_nl_solver_reduced_;  // N-1 次元

    /* callback スレッドが書き、plan スレッドが読む。要保護 */
    std::mutex fix_mutex_;
    bool   fix_gimbal_enabled_;
    double fix_gimbal_target_;

    /* plan スレッド専用 */
    double fix_gimbal_current_;    // レート制限をかけた実効固定角
    int    fix_gimbal_idx_;        // control_gimbal_names_ 内での位置（無ければ -1）
    double fix_gimbal_slew_rate_;  // [rad/s]
    double plan_du_;               // 1 / plan_freq
    std::string fix_gimbal_name_;

    /* plan() 冒頭で確定させ、nlopt 評価中は不変とみなすスナップショット */
    bool   active_fix_enabled_;
    int    active_fix_idx_;
    double active_fix_angle_;

    double last_fc_t_min_;

    void setupSolver(boost::shared_ptr<nlopt::opt> solver);
    void fixedGimbalCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    /* ================= ★ 固定ジンバル関連（ここまで） ================= */
  };
};