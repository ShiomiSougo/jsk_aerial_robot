#include <hydrus_xi/hydrus_xi_under_actuated_navigation.h>
#include <std_msgs/Float64MultiArray.h>

using namespace aerial_robot_navigation;

/* ============================================================================
 *  【改造の要点】大本のコードからの差分は以下の 3 点のみ。
 *
 *  1) 特定のジンバル（既定 "gimbal1"）を nlopt の最適化変数から外し、
 *     外部から与えた固定角として扱う。最適化次元は N -> N-1 に落ちる。
 *
 *  2) 固定角は指令値へ「レート制限つき」で追従させる（fix_gimbal_slew_rate_）。
 *     いきなり 3.14 -> 0.5 に飛ばすとサーボと機体が暴れるため。
 *
 *  3) Python 側が状態を見て遷移できるよう、固定角の現在値と
 *     feasible control torque min を state トピックで publish する。
 *
 *  導入しないもの（意図的）:
 *    - 内部モーメント計算（computeExactInternalMoment 相当）
 *    - 目的関数へのモーメント・ソフトペナルティ
 *    - stabilityCheck の無効化 / 閾値のハードコード上書き
 *  これらは本改造では不要。psi_1 を固定した時点で内部モーメントは
 *  静止推力 lambda_1 のスカラー倍として一意に決まるため。
 * ========================================================================== */

namespace
{
  int cnt = 0;
  int invalid_cnt = 0;

  /* 自由変数 x -> 全ジンバル角に展開してロボットモデルを更新する */
  void applyGimbalAngles(HydrusXiUnderActuatedNavigator *planner, const std::vector<double> &x)
  {
    auto robot_model = planner->getRobotModelForPlan();
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();

    const std::vector<double> full = planner->composeGimbalAngles(x);
    for(int i = 0; i < full.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = full.at(i);

    robot_model->updateRobotModel(joint_positions);
  }

  double maximizeFCTMin(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    applyGimbalAngles(planner, x);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        std::stringstream ss;
        for(const auto& angle: x) ss << angle << ", ";
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals: " << ss.str() << " (cnt: " << invalid_cnt << ")");
        return 0;
      }

    invalid_cnt = 0;

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());
    if(variant < 1e-6) variant = 1e-6; // 0 除算防止（推力が完全に均等な場合）

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()
         + planner->getForceVariantWeight() / variant
         + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();
  }

  double maximizeMinYawTorque(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    applyGimbalAngles(planner, x);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        if(planner->getPlanVerbose()) ROS_WARN("nlopt, robot stability is invalid (cnt: %d)", invalid_cnt);
        return 0;
      }
    else
      {
        invalid_cnt = 0;

        /* 1. calculate the max and min yaw torque by LP */
        Eigen::VectorXd gradient = robot_model->calcWrenchMatrixOnCoG().row(5).transpose();
        Eigen::VectorXd max_u, min_u;
        double max_yaw = 0, min_yaw = 0;

        planner->getYawRangeLPSolver().updateGradient(gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("can not calculate the min u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            min_u = planner->getYawRangeLPSolver().getSolution();
            min_yaw = (gradient.transpose() * min_u)(0);
            if(min_yaw > 0)
              {
                ROS_WARN("the min yaw is positive: %f", min_yaw);
                min_yaw = 0;
              }
          }

        Eigen::VectorXd reverse_gradient = - gradient;
        planner->getYawRangeLPSolver().updateGradient(reverse_gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("can not calculate the max u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            max_u = planner->getYawRangeLPSolver().getSolution();
            max_yaw = (gradient.transpose() * max_u)(0);
          }

        planner->setMaxMinYaw(std::min(max_yaw, -min_yaw));
      }

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());
    if(variant < 1e-6) variant = 1e-6;

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()
         + planner->getForceVariantWeight() / variant
         + planner->getYawTorqueWeight() * planner->getMaxMinYaw();
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

  /* [-pi, pi] へ正規化 */
  double normalizeAngle(double a)
  {
    while(a >  M_PI) a -= 2 * M_PI;
    while(a < -M_PI) a += 2 * M_PI;
    return a;
  }
};

HydrusXiUnderActuatedNavigator::HydrusXiUnderActuatedNavigator():
    opt_gimbal_angles_(0),
    prev_opt_gimbal_angles_(0),
    max_min_yaw_(0),
    control_gimbal_names_(0),
    control_gimbal_indices_(0),
    fix_gimbal_enabled_(false),
    fix_gimbal_target_(0.0),
    fix_gimbal_current_(0.0),
    fix_gimbal_idx_(-1),
    active_fix_enabled_(false),
    active_fix_idx_(-1),
    active_fix_angle_(0.0),
    last_fc_t_min_(0.0)
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

  robot_model_for_plan_ = boost::make_shared<HydrusTiltedRobotModel>(); // for planning, not the real robot model

  rosParamInit();

  gimbal_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

  /* ★ 固定ジンバル指令 / 状態フィードバック */
  fix_gimbal_cmd_sub_   = nh_.subscribe("fixed_gimbal_cmd", 1,
                                        &HydrusXiUnderActuatedNavigator::fixedGimbalCmdCallback, this);
  fix_gimbal_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fixed_gimbal_state", 1);

  if(nh.hasParam("control_gimbal_names"))
    {
      nh.getParam("control_gimbal_names", control_gimbal_names_);
    }
  else
    {
      ROS_INFO("load control gimbal list from robot model");
      for(const auto& name: robot_model->getJointNames())
        {
          if(name.find("gimbal") != std::string::npos)
            {
              control_gimbal_names_.push_back(name);
              ROS_INFO_STREAM("add " << name);
            }
        }
    }

  /* ★ 固定対象ジンバルの位置を control_gimbal_names_ の中から名前で引く
   *   （添字 0 が gimbal1 とは限らないため、順序に依存させない） */
  fix_gimbal_idx_ = -1;
  for(int i = 0; i < control_gimbal_names_.size(); i++)
    if(control_gimbal_names_.at(i) == fix_gimbal_name_) fix_gimbal_idx_ = i;

  if(fix_gimbal_idx_ < 0)
    ROS_WARN_STREAM("fix_gimbal_name '" << fix_gimbal_name_
                    << "' is not in control_gimbal_names. gimbal fixing will be disabled.");
  else
    ROS_INFO_STREAM("fixable gimbal: " << fix_gimbal_name_ << " (index " << fix_gimbal_idx_ << ")");

  const int n = control_gimbal_names_.size();

  /* ★ ソルバを 2 本用意する。
   *   full    : 全ジンバルを最適化（従来どおり）
   *   reduced : 固定ジンバルを除く N-1 個を最適化
   *   nlopt は lb == ub の変数を持つと COBYLA の初期単体が退化するため、
   *   次元そのものを落とす方が安全。 */
  vectoring_nl_solver_full_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, n);
  setupSolver(vectoring_nl_solver_full_);

  if(fix_gimbal_idx_ >= 0 && n >= 2)
    {
      vectoring_nl_solver_reduced_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, n - 1);
      setupSolver(vectoring_nl_solver_reduced_);
    }

  /* linear optimization for yaw range */
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

  plan_thread_ = std::thread(boost::bind(&HydrusXiUnderActuatedNavigator::threadFunc, this));
}

/* 目的関数・制約・収束条件の設定（full / reduced で共通） */
void HydrusXiUnderActuatedNavigator::setupSolver(boost::shared_ptr<nlopt::opt> solver)
{
  if(maximize_yaw_)
    {
      solver->set_max_objective(maximizeMinYawTorque, this);
      solver->add_inequality_constraint(fcTMinConstraint, this, 1e-8);
    }
  else
    solver->set_max_objective(maximizeFCTMin, this);

  solver->add_inequality_constraint(baselinkRotConstraint, this, 1e-8);

  solver->set_xtol_rel(1e-4);
  solver->set_maxeval(1000);
}

/* ★ 自由変数ベクトル x を、全ジンバル角ベクトルへ展開する。
 *   固定モードでなければ x をそのまま返す。 */
std::vector<double> HydrusXiUnderActuatedNavigator::composeGimbalAngles(const std::vector<double>& x)
{
  if(!active_fix_enabled_) return x;

  const int n = control_gimbal_names_.size();
  std::vector<double> full(n, 0.0);
  for(int i = 0, j = 0; i < n; i++)
    {
      if(i == active_fix_idx_) full.at(i) = active_fix_angle_;
      else                     full.at(i) = x.at(j++);
    }
  return full;
}

void HydrusXiUnderActuatedNavigator::fixedGimbalCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(msg->data.size() < 2) return;
  if(fix_gimbal_idx_ < 0 || !vectoring_nl_solver_reduced_)
    {
      ROS_WARN_THROTTLE(1.0, "fixed_gimbal_cmd received but gimbal fixing is unavailable");
      return;
    }

  std::lock_guard<std::mutex> lock(fix_mutex_);
  bool enable = (msg->data[0] > 0.5);

  if(enable != fix_gimbal_enabled_)
    ROS_INFO("[navi] gimbal fix mode: %s (target %.3f rad)",
             enable ? "ON" : "OFF", msg->data[1]);

  fix_gimbal_enabled_ = enable;
  fix_gimbal_target_  = normalizeAngle(msg->data[1]);
}

void HydrusXiUnderActuatedNavigator::threadFunc()
{
  double plan_freq;
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<double>(navi_nh, "plan_freq", plan_freq, 20.0);
  ros::Rate loop_rate(plan_freq);
  plan_du_ = 1.0 / plan_freq;

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
  joint_positions_for_plan_ = robot_model_->getJointPositions(); // real

  if(joint_positions_for_plan_.rows() == 0) return false;

  // initialize from the normal shape
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

  const int n = control_gimbal_indices_.size();

  /* ---- 初回のヒューリスティック初期値（大本のまま） ---------------------- */
  bool first_run = (opt_gimbal_angles_.size() == 0);
  if(first_run)
    {
      opt_gimbal_angles_.resize(n, 0);

      if(n == robot_model_->getRotorNum())
        {
          for(int i = 0; i < n; i++)
            if(i % 2 == 0) opt_gimbal_angles_.at(i) = M_PI;

          if(singular_form && robot_model_->getRotorNum() == 4)
            {
              opt_gimbal_angles_.at(0) =   M_PI / 2;
              opt_gimbal_angles_.at(1) = - M_PI / 2;
              opt_gimbal_angles_.at(2) = - M_PI / 2;
              opt_gimbal_angles_.at(3) =   M_PI / 2;
            }
        }
    }

  /* ---- ★ 固定ジンバルの状態更新（レート制限つき） ------------------------ */
  bool enable; double target;
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    enable = fix_gimbal_enabled_;
    target = fix_gimbal_target_;
  }

  if(enable && !active_fix_enabled_)
    {
      /* 固定モードに入った瞬間：現在角から出発する */
      fix_gimbal_current_ = opt_gimbal_angles_.at(fix_gimbal_idx_);
      ROS_INFO("[navi] start slewing %s: %.3f -> %.3f rad",
               fix_gimbal_name_.c_str(), fix_gimbal_current_, target);
    }

  if(enable)
    {
      /* 最短回り方向へ、fix_gimbal_slew_rate_ [rad/s] で追従 */
      double err = normalizeAngle(target - fix_gimbal_current_);
      double step = fix_gimbal_slew_rate_ * plan_du_;
      if(fabs(err) < step) fix_gimbal_current_ = target;
      else                 fix_gimbal_current_ += (err > 0 ? step : -step);
      fix_gimbal_current_ = normalizeAngle(fix_gimbal_current_);
    }

  active_fix_enabled_ = enable;
  active_fix_idx_     = fix_gimbal_idx_;
  active_fix_angle_   = fix_gimbal_current_;

  boost::shared_ptr<nlopt::opt> solver =
    active_fix_enabled_ ? vectoring_nl_solver_reduced_ : vectoring_nl_solver_full_;

  /* ---- 自由変数の抽出と探索範囲 ------------------------------------------ */
  std::vector<double> x;                       // 最適化にかける自由変数
  for(int i = 0; i < n; i++)
    {
      if(active_fix_enabled_ && i == active_fix_idx_) continue;
      x.push_back(opt_gimbal_angles_.at(i));
    }

  std::vector<double> lb(x.size(), -M_PI), ub(x.size(), M_PI);

  if(!first_run)
    {
      double delta_angle = gimbal_delta_angle_;
      if(!robot_model_for_plan_->stabilityCheck(false)) delta_angle = M_PI; // reset

      for(int i = 0; i < x.size(); i++)
        {
          lb.at(i) = x.at(i) - delta_angle;
          ub.at(i) = x.at(i) + delta_angle;
        }
    }

  solver->set_lower_bounds(lb);
  solver->set_upper_bounds(ub);

  /* ---- 最適化 ------------------------------------------------------------ */
  double start_time = ros::Time::now().toSec();
  double max_f = 0;
  try
    {
      solver->optimize(x, max_f);

      /* 自由変数を全体ベクトルへ書き戻す */
      opt_gimbal_angles_ = composeGimbalAngles(x);

      last_fc_t_min_ = robot_model_for_plan_->getFeasibleControlTMin();

      if(plan_verbose_)
        {
          double roll, pitch, yaw;
          robot_model_for_plan_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

          std::cout << "nlopt: " << std::setprecision(7)
                    << ros::Time::now().toSec() - start_time << "[sec], cnt: " << cnt
                    << (active_fix_enabled_ ? " [FIXED]" : " [FREE]");
          std::cout << ", gimbals: ";
          for(auto it: opt_gimbal_angles_) std::cout << std::setprecision(5) << it << " ";
          std::cout << ", max min yaw: " << max_min_yaw_;
          std::cout << ", fc t min: " << last_fc_t_min_;
          std::cout << ", attitude: [" << roll << ", " << pitch;
          std::cout << "], force: [" << robot_model_for_plan_->getStaticThrust().transpose();
          std::cout << "]" << std::endl;
        }

      cnt = 0;
      invalid_cnt = 0;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }

  /* ---- ジンバル角の publish ---------------------------------------------- */
  sensor_msgs::JointState gimbal_msg;
  gimbal_msg.header.stamp = ros::Time::now();

  for(int i = 0; i < n; i++)
    {
      gimbal_msg.name.push_back(control_gimbal_names_.at(i));
      gimbal_msg.position.push_back(opt_gimbal_angles_.at(i));
    }
  gimbal_ctrl_pub_.publish(gimbal_msg);

  /* ---- ★ 状態フィードバック（Python 側の遷移判定に使う） ----------------- */
  std_msgs::Float64MultiArray state_msg;
  state_msg.data.resize(5);
  state_msg.data[0] = active_fix_enabled_ ? 1.0 : 0.0;
  state_msg.data[1] = target;                                   // 指令値
  state_msg.data[2] = active_fix_enabled_ ? active_fix_angle_
                    : (fix_gimbal_idx_ >= 0 ? opt_gimbal_angles_.at(fix_gimbal_idx_) : 0.0);
  state_msg.data[3] = last_fc_t_min_;                           // feasible control torque min
  state_msg.data[4] = active_fix_enabled_
                    ? fabs(normalizeAngle(target - active_fix_angle_)) : 0.0;
  fix_gimbal_state_pub_.publish(state_msg);

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

  /* ★ 追加パラメータ */
  getParam<std::string>(navi_nh, "fix_gimbal_name", fix_gimbal_name_, std::string("gimbal1"));
  getParam<double>(navi_nh, "fix_gimbal_slew_rate", fix_gimbal_slew_rate_, 0.5); // [rad/s]
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HydrusXiUnderActuatedNavigator, aerial_robot_navigation::BaseNavigator);