// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, JSK Lab
 * All rights reserved.
 * ...
 *********************************************************************/

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <algorithm>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <nlopt.hpp>
#include <OsqpEigen/OsqpEigen.h>
// ===== 内部モーメント指令用 =====
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h> 

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

    // ===== 内部モーメント制御用パブリックアクセッサー & メソッド =====
    inline int getTargetJointIndex() const { return target_joint_index_; }
    inline double getTauDesTarget() const { return tau_des_target_; }
    inline bool hasMomentCommand() const { return has_moment_command_; }
    
    double computeExactInternalMoment(
        const std::vector<double>& x,
        const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr);

  private:
    // ★ 【修正】各ジンバルへの個別コマンド送信に変更
    std::vector<ros::Publisher> gimbal_ctrl_pubs_; 
    
    std::thread plan_thread_;
    boost::shared_ptr<HydrusTiltedRobotModel> robot_model_for_plan_;
    OsqpEigen::Solver yaw_range_lp_solver_;
    boost::shared_ptr<nlopt::opt> vectoring_nl_solver_;

    KDL::JntArray joint_positions_for_plan_;
    std::vector<std::string> control_gimbal_names_;
    std::vector<int> control_gimbal_indices_;
    double max_min_yaw_;

    bool plan_verbose_;
    bool maximize_yaw_;
    double force_norm_weight_;
    double force_variant_weight_;
    double yaw_torque_weight_;
    double fc_t_min_weight_;
    double baselink_rot_thresh_;
    double fc_t_min_thresh_;
    double gimbal_delta_angle_;

    std::vector<double> opt_gimbal_angles_, prev_opt_gimbal_angles_;

    void threadFunc();
    bool plan();

    void rosParamInit() override;

    // ===== 内部モーメント制御用メンバ変数 =====
    int target_joint_index_;
    double tau_des_target_;
    bool has_moment_command_;

    ros::Subscriber moment_command_sub_;

    void momentCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    std::vector<double> extractThrustsFromOptVars(
        const std::vector<double>& x,
        const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr);
        
    std::vector<double> extractGimbalsFromOptVars(
        const std::vector<double>& x);
  };
} // namespace aerial_robot_navigation