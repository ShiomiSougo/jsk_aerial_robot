#include <hydrus_xi/hydrus_xi_under_actuated_navigation.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <cmath>

using namespace aerial_robot_navigation;

namespace
{
  int cnt = 0;
  int invalid_cnt = 0;

  // ====================================================================
  // ★ 【修正7-c・最重要】非対称モーメントペナルティ
  //
  // 【問題】
  //   従来は対称な二乗誤差 w_tau * diff^2 のみだったため、
  //   computeExactInternalMoment() が持つロータ反トルク由来の
  //   構造的な負バイアス（mz_local = kappa*f*dir*cos(beta),
  //   dir が i=1,2,3 で -,+,- となり総和が負に偏る）に負けて、
  //   tau_des > 0 のときに tau が負のまま最適解になっていた。
  //   結果、正方向変形（0.3 -> 0.9）で joint1 の動きが逆転していた。
  //
  // 【対策】
  //   (1) 符号違反（tau が tau_des と逆向き）に、二乗＋線形の
  //       極端に重いペナルティを課す。線形項は COBYLA（微分不要法）が
  //       負側の谷から抜け出すための「勾配の壁」として機能する。
  //   (2) 正しい向きへのオーバーシュートは減免する。関節を動かすのに
  //       必要なのは「正確な tau」ではなく「正しい向きの十分な tau」。
  // ====================================================================
  double computeMomentPenalty(HydrusXiUnderActuatedNavigator *planner,
                              const std::vector<double> &x,
                              boost::shared_ptr<HydrusTiltedRobotModel> robot_model)
  {
      // has_moment_command_ フラグが立たない限りペナルティ無効
      if (!planner->hasMomentCommand() || planner->getTargetJointIndex() < 0) {
          return 0.0;
      }

      const double tau     = planner->computeExactInternalMoment(x, robot_model);
      const double tau_des = planner->getTauDesTarget();
      const double diff    = tau - tau_des;

      const double w_tau            = 2000.0;   // 基本の二乗誤差重み
      const double w_sign_quad      = 2.0e5;    // 符号違反の二乗ペナルティ
      const double w_sign_lin       = 1.0e4;    // 符号違反の線形ペナルティ（勾配の壁）
      const double overshoot_relief = 0.90;     // 正方向オーバーシュートの減免率

      double penalty = w_tau * diff * diff;

      if (std::fabs(tau_des) > 1e-6) {
          const double s = (tau_des > 0.0) ? 1.0 : -1.0;

          // ★ 符号が逆（tau が tau_des と反対向き）なら極端に重いペナルティ
          const double violation = -s * tau;   // >0 で符号違反
          if (violation > 0.0) {
              penalty += w_sign_quad * violation * violation;
              penalty += w_sign_lin  * violation;
          }

          // ★ 正しい向きに行き過ぎた分は軽く（動かすのに害はない）
          const double overshoot = s * tau - s * tau_des;
          if (overshoot > 0.0) {
              penalty -= overshoot_relief * w_tau * overshoot * overshoot;
          }
      }

      ROS_INFO_THROTTLE(0.5,
          "[MomentPenalty] joint%d: tau=%.4f, tau_des=%.4f, diff=%.4f, penalty=%.3f",
          planner->getTargetJointIndex() + 1, tau, tau_des, diff, penalty);

      return penalty;
  }

  double maximizeFCTMin(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(false && !robot_model->stabilityCheck(planner->getPlanVerbose()))
    {
        invalid_cnt ++;
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals (cnt: " << invalid_cnt << ")");
        return 0;
    }

    invalid_cnt = 0;

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variance_val = 0;

    for(int i = 0; i < force_v.size(); i++)
      variance_val += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variance_val = sqrt(variance_val / force_v.size());

    double objective_base = planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()
                          + planner->getForceVariantWeight() / variance_val
                          + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();

    // ★ 【修正7-d】モーメント指令中は基本項を弱め、モーメント追従を支配的にする。
    //    基本項とペナルティが同オーダーで競合すると、COBYLA が「基本項を稼ぐ」
    //    折衷解に落ち、tau の符号が反転したままになるため。
    //    姿勢と可制御性は baselinkRotConstraint / fcTMinConstraint が担保する。
    const double base_scale = planner->hasMomentCommand() ? 0.05 : 1.0;

    return base_scale * objective_base - computeMomentPenalty(planner, x, robot_model);
  }

  double maximizeMinYawTorque(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(false && !robot_model->stabilityCheck(planner->getPlanVerbose()))
    {
        invalid_cnt ++;
        if(planner->getPlanVerbose()) ROS_WARN("nlopt, robot stability is invalid (cnt: %d)", invalid_cnt);
        return 0;
    }

    invalid_cnt = 0;

    Eigen::VectorXd gradient = robot_model->calcWrenchMatrixOnCoG().row(5).transpose();
    Eigen::VectorXd max_u, min_u;
    double max_yaw, min_yaw;

    planner->getYawRangeLPSolver().updateGradient(gradient);
    if(!planner->getYawRangeLPSolver().solve())
    {
        ROS_ERROR("can not calcualte the min u by LP");
        planner->setMaxMinYaw(0);
    }
    else
    {
        min_u = planner->getYawRangeLPSolver().getSolution();
        min_yaw = (gradient.transpose() * min_u)(0);
        if(min_yaw > 0) { min_yaw = 0; }
    }

    Eigen::VectorXd reverse_gradient = - gradient;
    planner->getYawRangeLPSolver().updateGradient(reverse_gradient);
    if(!planner->getYawRangeLPSolver().solve())
    {
        ROS_ERROR("can not calcualte the max u by LP");
        planner->setMaxMinYaw(0);
    }
    else
    {
        max_u = planner->getYawRangeLPSolver().getSolution();
        max_yaw = (gradient.transpose() * max_u)(0);
    }

    planner->setMaxMinYaw(std::min(max_yaw, -min_yaw));

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variance_val = 0;

    for(int i = 0; i < force_v.size(); i++)
      variance_val += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variance_val = sqrt(variance_val / force_v.size());

    double objective_base = planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()
                          + planner->getForceVariantWeight() / variance_val
                          + planner->getYawTorqueWeight() * planner->getMaxMinYaw();

    // ★ 【修正7-d】モーメント指令中は基本項を弱める（maximizeFCTMin と同様）
    const double base_scale = planner->hasMomentCommand() ? 0.05 : 1.0;

    return base_scale * objective_base - computeMomentPenalty(planner, x, robot_model);
  }

  double baselinkRotConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto baselink_rot = planner->getRobotModelForPlan()->getCogDesireOrientation<Eigen::Matrix3d>();

    double ez_x = baselink_rot(0,2);
    double ez_y = baselink_rot(1,2);
    double ez_z = baselink_rot(2,2);
    double angle = atan2(sqrt(ez_x* ez_x + ez_y * ez_y), fabs(ez_z));

    return angle - planner->getBaselinkRotThresh();
  }

  double fcTMinConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    return planner->getFCTMinThresh() - planner->getRobotModelForPlan()->getFeasibleControlTMin();
  }
}

HydrusXiUnderActuatedNavigator::HydrusXiUnderActuatedNavigator():
    opt_gimbal_angles_(0),
    prev_opt_gimbal_angles_(0),
    max_min_yaw_(0),
    control_gimbal_names_(0),
    control_gimbal_indices_(0),
    target_joint_index_(-1),
    tau_des_target_(0.0),
    has_moment_command_(false)
{
}

HydrusXiUnderActuatedNavigator::~HydrusXiUnderActuatedNavigator()
{
  plan_thread_.join();
}

void HydrusXiUnderActuatedNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                double loop_du)
{
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  robot_model_for_plan_ = boost::make_shared<HydrusTiltedRobotModel>();

  rosParamInit();

  target_joint_index_ = -1;
  tau_des_target_ = 0.0;
  has_moment_command_ = false;

  moment_command_sub_ = nh_.subscribe(
      "/hydrus_xi/target_internal_moment",
      1,
      &HydrusXiUnderActuatedNavigator::momentCommandCallback,
      this
  );

  if(nh.hasParam("control_gimbal_names"))
    {
      nh.getParam("control_gimbal_names", control_gimbal_names_);
    }
  else
    {
      for(const auto& name: robot_model->getJointNames())
        {
          if(name.find("gimbal") != std::string::npos)
            {
              control_gimbal_names_.push_back(name);
            }
        }
    }

  gimbal_ctrl_pubs_.resize(control_gimbal_names_.size());
  for(int i = 0; i < control_gimbal_names_.size(); i++)
    {
      std::string topic = "/hydrus_xi/servo_controller/gimbals/controller" + std::to_string(i+1) + "/simulation/command";
      gimbal_ctrl_pubs_[i] = nh_.advertise<std_msgs::Float64>(topic, 1);
    }

  vectoring_nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, control_gimbal_names_.size());
  if(maximize_yaw_)
    {
      vectoring_nl_solver_->set_max_objective(maximizeMinYawTorque, this);
      vectoring_nl_solver_->add_inequality_constraint(fcTMinConstraint, this, 1e-8);
    }
  else
    vectoring_nl_solver_->set_max_objective(maximizeFCTMin, this);

  vectoring_nl_solver_->add_inequality_constraint(baselinkRotConstraint, this, 1e-8);

  vectoring_nl_solver_->set_xtol_rel(1e-4);
  vectoring_nl_solver_->set_maxeval(50);

  double rotor_num = robot_model->getRotorNum();

  yaw_range_lp_solver_.settings()->setVerbosity(false);
  yaw_range_lp_solver_.settings()->setWarmStart(true);
  yaw_range_lp_solver_.data()->setNumberOfVariables(rotor_num);
  yaw_range_lp_solver_.data()->setNumberOfConstraints(rotor_num);

  Eigen::SparseMatrix<double> hessian;
  hessian.resize(rotor_num, rotor_num);

  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(rotor_num);
  Eigen::SparseMatrix<double> linear_cons;
  linear_cons.resize(rotor_num, rotor_num);
  for(int i = 0; i < linear_cons.cols(); i++) linear_cons.insert(i,i) = 1;

  Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustLowerLimit();
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustUpperLimit();

  yaw_range_lp_solver_.data()->setHessianMatrix(hessian);
  yaw_range_lp_solver_.data()->setGradient(gradient);
  yaw_range_lp_solver_.data()->setLinearConstraintsMatrix(linear_cons);
  yaw_range_lp_solver_.data()->setLowerBound(lower_bound);
  yaw_range_lp_solver_.data()->setUpperBound(upper_bound);

  if(!yaw_range_lp_solver_.initSolver())
    throw std::runtime_error("can not init LP solver based on osqp");

  opt_gimbal_angles_.clear();
  prev_opt_gimbal_angles_.clear();

  plan_thread_ = std::thread(boost::bind(&HydrusXiUnderActuatedNavigator::threadFunc, this));
}

void HydrusXiUnderActuatedNavigator::threadFunc()
{
  double plan_freq;
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<double>(navi_nh, "plan_freq", plan_freq, 20.0);
  ros::Rate loop_rate(plan_freq);

  double plan_init_sleep;
  getParam<double>(navi_nh, "plan_init_sleep", plan_init_sleep, 2.0);
  ros::Duration(plan_init_sleep).sleep();

  while(ros::ok())
    {
      plan();
      loop_rate.sleep();
    }
}

bool HydrusXiUnderActuatedNavigator::plan()
{
  joint_positions_for_plan_ = robot_model_->getJointPositions();

  if(joint_positions_for_plan_.rows() == 0) return false;

  bool singular_form = true;
  if(control_gimbal_indices_.size() == 0)
    {
      const auto& joint_names = robot_model_->getJointNames();
      const auto& joint_indices = robot_model_->getJointIndices();

      for(int i = 0; i < joint_names.size(); i++)
        {
          if(joint_names.at(i).find("joint") != std::string::npos)
            {
              if(fabs(joint_positions_for_plan_(joint_indices.at(i))) > 0.2) singular_form = false;
            }
        }

      for(const auto& name: control_gimbal_names_)
        control_gimbal_indices_.push_back(robot_model_->getJointIndexMap().at(name));
    }

  std::vector<double> lb(control_gimbal_indices_.size(), - M_PI);
  std::vector<double> ub(control_gimbal_indices_.size(), M_PI);

  if(opt_gimbal_angles_.size() != 0)
    {
      double delta_angle = gimbal_delta_angle_;

      if(!robot_model_for_plan_->stabilityCheck(false))
        {
          delta_angle = M_PI;
        }

      // ★ 【修正7-e】モーメント指令中は探索範囲を広げ、
      //    符号が逆の局所解（負トルクの谷）から脱出できるようにする。
      if(has_moment_command_ && target_joint_index_ >= 0)
        {
          delta_angle = M_PI;
        }

      for(int i = 0; i < opt_gimbal_angles_.size(); i++)
         {
           lb.at(i) = opt_gimbal_angles_.at(i) - delta_angle;
           ub.at(i) = opt_gimbal_angles_.at(i) + delta_angle;
         }
    }
  else
    {
      opt_gimbal_angles_.resize(control_gimbal_indices_.size(), 0);

      if(control_gimbal_indices_.size() == robot_model_->getRotorNum())
        {
          for(int i = 0; i < control_gimbal_indices_.size(); i++)
            {
              if(i%2 == 0) opt_gimbal_angles_.at(i) = M_PI;
            }

          if(singular_form && robot_model_->getRotorNum() == 4)
            {
              opt_gimbal_angles_.at(0) = M_PI / 2;
              opt_gimbal_angles_.at(1) = - M_PI / 2;
              opt_gimbal_angles_.at(2) = - M_PI / 2;
              opt_gimbal_angles_.at(3) = M_PI / 2;
            }
        }
    }

  vectoring_nl_solver_->set_lower_bounds(lb);
  vectoring_nl_solver_->set_upper_bounds(ub);

  if (has_moment_command_ && target_joint_index_ >= 0) {
      double current_tau = computeExactInternalMoment(opt_gimbal_angles_, robot_model_for_plan_);
      ROS_INFO_THROTTLE(0.5, "plan() before optimize: joint%d current_tau=%.4f, target_tau=%.4f, gimbal_angles=[%.4f, %.4f, %.4f, %.4f]",
          target_joint_index_ + 1, current_tau, tau_des_target_,
          opt_gimbal_angles_.size() > 0 ? opt_gimbal_angles_[0] : 0,
          opt_gimbal_angles_.size() > 1 ? opt_gimbal_angles_[1] : 0,
          opt_gimbal_angles_.size() > 2 ? opt_gimbal_angles_[2] : 0,
          opt_gimbal_angles_.size() > 3 ? opt_gimbal_angles_[3] : 0);
  }

  double start_time = ros::Time::now().toSec();
  double max_f = 0;
  try
    {
      nlopt::result result = vectoring_nl_solver_->optimize(opt_gimbal_angles_, max_f);

      double roll,pitch,yaw;
      robot_model_for_plan_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

      if(prev_opt_gimbal_angles_.size() == 0) prev_opt_gimbal_angles_ = opt_gimbal_angles_;

      // ★ 【修正7-f】最適化後、tau の符号が指令と逆なら警告を出す。
      //    到達不能なのか局所解なのかを切り分けるための診断ログ。
      if(has_moment_command_ && target_joint_index_ >= 0)
        {
          double tau_after = computeExactInternalMoment(opt_gimbal_angles_, robot_model_for_plan_);
          if(std::fabs(tau_des_target_) > 1e-6 && tau_after * tau_des_target_ < 0.0)
            {
              ROS_WARN_THROTTLE(1.0,
                  "[MomentSign] joint%d: 符号不一致! tau=%.4f vs tau_des=%.4f "
                  "(局所解か到達不能。gimbal_delta_angle / w_sign_* を要調整)",
                  target_joint_index_ + 1, tau_after, tau_des_target_);
            }
        }

      if(plan_verbose_)
        {
          std::cout << "nlopt: " << std::setprecision(7)
                    << ros::Time::now().toSec() - start_time  <<  "[sec], cnt: " << cnt;
          std::cout << ", found optimal gimbal angles: ";
          for(auto it: opt_gimbal_angles_) std::cout << std::setprecision(5) << it << " ";
          std::cout << ", max min yaw: " << max_min_yaw_;
          std::cout << ", fc t min: " << robot_model_for_plan_->getFeasibleControlTMin();
          std::cout << ", force: [" << robot_model_for_plan_->getStaticThrust().transpose() << "]" << std::endl;
        }

      cnt = 0;
      invalid_cnt = 0;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }

  for(int i = 0; i < control_gimbal_indices_.size(); i++)
    {
      std_msgs::Float64 msg;
      msg.data = opt_gimbal_angles_.at(i);
      if (i < gimbal_ctrl_pubs_.size())
        {
          gimbal_ctrl_pubs_[i].publish(msg);
        }
    }

  prev_opt_gimbal_angles_ = opt_gimbal_angles_;

  return true;
}

void HydrusXiUnderActuatedNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<bool>(navi_nh, "plan_verbose", plan_verbose_, false);
  getParam<bool>(navi_nh, "maximize_yaw", maximize_yaw_, false);
  getParam<double>(navi_nh, "gimbal_delta_angle", gimbal_delta_angle_, 0.2);
  getParam<double>(navi_nh, "force_norm_rate", force_norm_weight_, 2.0);
  getParam<double>(navi_nh, "force_variant_rate", force_variant_weight_, 0.01);
  getParam<double>(navi_nh, "yaw_torque_weight", yaw_torque_weight_, 1.0);
  getParam<double>(navi_nh, "fc_t_min_weight", fc_t_min_weight_, 1.0);
  getParam<double>(navi_nh, "baselink_rot_thresh", baselink_rot_thresh_, 0.02);
  getParam<double>(navi_nh, "fc_t_min_thresh", fc_t_min_thresh_, 2.0);

  baselink_rot_thresh_ = 0.08;
  fc_t_min_thresh_ = 0.2;
  gimbal_delta_angle_ = 0.5;
}

void HydrusXiUnderActuatedNavigator::momentCommandCallback(
    const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // ====================================================================
  // ★ 【修正1】モーメント命令の解析
  // msg->data[0] = target_joint_index (-1=なし, 0,1,2,...=対象関節)
  // msg->data[1] = tau_des_target (目標モーメント [N・m])
  //
  // target_joint_index_ = -1 のとき has_moment_command_ = false
  // それ以外のとき has_moment_command_ = true
  // ====================================================================
  if (msg->data.size() < 2) return;

  int new_target_joint_index = static_cast<int>(msg->data[0]);
  double new_tau_des_target = msg->data[1];

  // target_joint_indexが-1（センチネル値）なら、モーメント制御OFF
  if (new_target_joint_index == -1) {
    target_joint_index_ = -1;
    tau_des_target_ = 0.0;
    has_moment_command_ = false;
    ROS_INFO_THROTTLE(1.0, "[HydrusXiUnderActuatedNavigator] 📭 モーメント制御 OFF (target_joint_index = -1)");
  } else {
    target_joint_index_ = new_target_joint_index;
    tau_des_target_ = new_tau_des_target;
    has_moment_command_ = true;
    ROS_INFO_THROTTLE(0.5, "[HydrusXiUnderActuatedNavigator] 📨 モーメント命令受信 ON: joint_idx=%d, tau_target=%.4f",
                      target_joint_index_, tau_des_target_);
  }
}

double HydrusXiUnderActuatedNavigator::computeExactInternalMoment(
    const std::vector<double>& gimbal_angles,
    const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr)
{
  if (!robot_model_ptr || target_joint_index_ < 0) return 0.0;

  Eigen::VectorXd thrusts = robot_model_ptr->getStaticThrust();
  int num_rotors = thrusts.size();
  int num_joints = num_rotors - 1;

  if (target_joint_index_ >= num_joints) return 0.0;

  std::vector<double> q(num_joints, 0.0);
  try {
    const auto& joint_map = robot_model_ptr->getJointIndexMap();
    for (int i = 0; i < num_joints; ++i) {
        std::string joint_name = "joint" + std::to_string(i + 1);
        q[i] = robot_model_ptr->getJointPositions()(joint_map.at(joint_name));
    }
  } catch (const std::exception& e) {
    return 0.0;
  }

  const double beta = 0.34906585;
  const double kappa = 0.0182;
  const double dx = 0.3016;
  const double L = 0.6;

  std::vector<Eigen::Vector3d> P_L(num_rotors);
  std::vector<double> theta(num_rotors);

  P_L[0] = Eigen::Vector3d(0, 0, 0);
  theta[0] = 0.0;

  for (int i = 1; i < num_rotors; ++i) {
    theta[i] = theta[i-1] + q[i-1];
    P_L[i] = P_L[i-1] + Eigen::Vector3d(L * std::cos(theta[i-1]), L * std::sin(theta[i-1]), 0.0);
  }

  Eigen::Vector3d P_joint = P_L[target_joint_index_ + 1];
  double tau_internal = 0.0;

  for (int i = target_joint_index_ + 1; i < num_rotors; ++i) {

    Eigen::Vector3d P_rot = P_L[i] + Eigen::Vector3d(dx * std::cos(theta[i]), dx * std::sin(theta[i]), 0.0);
    Eigen::Vector3d r = P_rot - P_joint;

    double f = thrusts(i);
    double psi = gimbal_angles[i];

    double dir = (i % 2 == 0) ? 1.0 : -1.0;
    double T_yaw = kappa * f * dir;

    double sin_b = std::sin(beta), cos_b = std::cos(beta);
    double sin_p = std::sin(psi),  cos_p = std::cos(psi);

    double fx_local = -f * sin_b * cos_p;
    double fy_local = -f * sin_b * sin_p;
    double mz_local = T_yaw * cos_b;

    double Fx_world = fx_local * std::cos(theta[i]) - fy_local * std::sin(theta[i]);
    double Fy_world = fx_local * std::sin(theta[i]) + fy_local * std::cos(theta[i]);

    double torque_from_force = r.x() * Fy_world - r.y() * Fx_world;
    tau_internal += (torque_from_force + mz_local);
  }

  return tau_internal;
}

std::vector<double> HydrusXiUnderActuatedNavigator::extractThrustsFromOptVars(
    const std::vector<double>& x,
    const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr)
{
  std::vector<double> thrusts;
  Eigen::VectorXd force_v = robot_model_ptr->getStaticThrust();
  for (int i = 0; i < force_v.size(); ++i) thrusts.push_back(force_v(i));
  return thrusts;
}

std::vector<double> HydrusXiUnderActuatedNavigator::extractGimbalsFromOptVars(
    const std::vector<double>& x)
{
  return x;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HydrusXiUnderActuatedNavigator, aerial_robot_navigation::BaseNavigator);