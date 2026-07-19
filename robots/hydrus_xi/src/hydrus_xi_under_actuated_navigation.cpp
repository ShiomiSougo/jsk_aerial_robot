#include <hydrus_xi/hydrus_xi_under_actuated_navigation.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

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
 *
 *  【診断用の追加（デバッグ専用、恒久対策ではない）】
 *  - plan_debug トピック: πリセット・ジンバル角ジャンプの検知
 *  - diagnoseGlobalMaxFCTMin(): リセット発生時のみ、gimbal2,3,4を
 *    広域グリッド探索し、「探索範囲不足」か「真の特異点」かを判定する
 *
 *  【★rev.12 追加】探索範囲の逐次拡大（エスカレーション）
 *  - 旧ロジックは「前周期の解が不安定 -> 次周期は delta=π で全開放」という
 *    "周期をまたぐ" 二値リセットだった。谷の中では毎周期 π で別々の局所解に
 *    飛びつき、検証されていないその解がそのまま publish され続けるという
 *    問題（LARGE GIMBAL JUMP の連鎖）が実験で確認された。
 *  - 新ロジックは plan() の 1 周期内で完結するリトライループにする：
 *      1) delta = gimbal_delta_angle_ (既定0.2rad) で出発点 x0 から最適化
 *      2) 得られた解が不安定 or fc_t_min が小さすぎれば、x を x0 に戻し、
 *         delta を escalation_factor 倍に広げて再度最適化
 *      3) 安定解が見つかるか、最大リトライ回数 or 時間予算に達するまで
 *         繰り返す
 *      4) それでも見つからない場合のみ、最終手段として広域グリッド探索
 *         (diagnoseGlobalMaxFCTMin) の結果を採用する
 *    実機へ publish されるのは、このループ内で検証済みの解のみ。
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

  /* ★ [rev.16 変更] 真の特異点かどうかを診断するための広域グリッドサーチ。
   *   現在のjoint角を保持したまま、自由変数（gimbal1固定時はgimbal2,3,4の
   *   n=3、gimbal1が自由な場合はgimbal1〜4のn=4）を [-pi, pi] の範囲で
   *   グリッド探索し、達成可能な fc_t_min の最大値を返す。
   *
   *   rev.12〜15までは n=3（gimbal1固定モード）専用に実装されており、
   *   gimbal1が自由な状態（JOINT23_SERVOステップ等、n=4）では診断自体を
   *   スキップして -1 を返す実装だった。この結果、n=4の状態でエスカレー
   *   ションが尽きても救済手段が一切なく、joint2,3の変形経路がS1/S2
   *   特異条件を横切った際にフォースランディングに至る事例が確認された
   *   （q=(0.5,-1,0.5)実験）。
   *
   *   rev.16でn=3固定の制約を撤廃し、任意の次元数nに対応できるよう
   *   一般化した。総評価点数が target_eval_budget 程度になるよう、
   *   次元数に応じて各軸の分割数（grid_steps）を自動調整する
   *   （n=3なら従来通り12分割相当、n=4ならおよそ6分割になる）。
   *
   *   x                 : 現在のnlopt自由変数（固定ジンバルを除いた角度ベクトル）
   *   target_eval_budget: 総評価点数の目安（既定1728 = 旧来のn=3,12分割相当）
   *
   *   注意: x, robot_model の状態は関数内で書き換わる。呼び出し側は
   *   戻り値を見た上で、必要なら best_x_out を採用解として使う。
   */
  double diagnoseGlobalMaxFCTMin(HydrusXiUnderActuatedNavigator *planner,
                                  const std::vector<double> &current_x,
                                  int target_eval_budget,
                                  std::vector<double> &best_x_out)
  {
    auto robot_model = planner->getRobotModelForPlan();
    const int n = static_cast<int>(current_x.size());

    best_x_out = current_x;
    double best_fc_t_min = -1.0;

    if(n < 1)
      {
        ROS_WARN_STREAM("diagnoseGlobalMaxFCTMin: invalid dimension n=" << n << ". skip diagnosis.");
        return -1.0;
      }

    /* 次元数nに応じて各軸の分割数を自動調整する。
     * grid_steps^n ≈ target_eval_budget となるように grid_steps を決める。
     * 最低3分割は確保する（1〜2分割では探索の意味がほぼ無いため）。 */
    int grid_steps = std::max(3, static_cast<int>(std::round(
      std::pow(static_cast<double>(target_eval_budget), 1.0 / static_cast<double>(n)))));

    std::vector<double> trial = current_x;
    std::vector<int> idx(n, 0);
    int evaluated = 0, valid = 0;

    while(true)
      {
        for(int d = 0; d < n; d++)
          trial[d] = -M_PI + 2 * M_PI * idx[d] / grid_steps;

        applyGimbalAngles(planner, trial);
        evaluated++;

        if(robot_model->stabilityCheck(false))
          {
            valid++;
            double fc_t_min = robot_model->getFeasibleControlTMin();
            if(fc_t_min > best_fc_t_min)
              {
                best_fc_t_min = fc_t_min;
                best_x_out = trial;
              }
          }

        /* n次元の桁上げ（オドメータ式カウンタ）でグリッド全点を走査する */
        int d = n - 1;
        while(d >= 0)
          {
            idx[d]++;
            if(idx[d] < grid_steps) break;
            idx[d] = 0;
            d--;
          }
        if(d < 0) break; // 全軸が一周した = 探索完了
      }

    std::stringstream best_x_ss;
    for(size_t i = 0; i < best_x_out.size(); i++) best_x_ss << best_x_out[i] << (i + 1 < best_x_out.size() ? ", " : "");

    ROS_WARN_STREAM("[navi][plan_debug] diagnoseGlobalMaxFCTMin: n=" << n << ", grid_steps=" << grid_steps
                    << " (" << evaluated << " points evaluated, " << valid << " stable). "
                    << "max fc_t_min = " << best_fc_t_min
                    << " at gimbal = [" << best_x_ss.str() << "]");

    return best_fc_t_min;
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
    fix_gimbal_slew_rate_(0.5),
    plan_du_(0.05),
    active_fix_enabled_(false),
    active_fix_idx_(-1),
    active_fix_angle_(0.0),
    last_fc_t_min_(0.0),
    last_invalid_cnt_(0),
    last_max_jump_(0.0)
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

  /* ★ [追加] 診断用トピック */
  plan_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("plan_debug", 1);

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

  if(joint_positions_for_plan_.rows() == 0)
    {
      ROS_ERROR_THROTTLE(0.5, "[navi][plan_debug] plan() EARLY RETURN: joint_positions_for_plan_ is empty");
      return false;
    }
    //追記：「plan()が早期returnしている」という仮説を検証
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

  /* 保険: reduced ソルバが無い構成では固定モードに入らない */
  if(enable && (fix_gimbal_idx_ < 0 || !vectoring_nl_solver_reduced_))
    {
      ROS_WARN_THROTTLE(1.0, "[navi] gimbal fixing unavailable, fall back to full optimization");
      enable = false;
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

  /* ---- 自由変数の抽出 ------------------------------------------------------ */
  std::vector<double> x;                       // 最適化にかける自由変数
  for(int i = 0; i < n; i++)
    {
      if(active_fix_enabled_ && i == active_fix_idx_) continue;
      x.push_back(opt_gimbal_angles_.at(i));
    }

  /* ============================================================================
   * ★ [rev.12 変更] 探索範囲の逐次拡大（エスカレーション）
   *
   *   旧ロジック（削除済み）:
   *     前周期の解が不安定 -> 次周期は delta=π で全開放、という
   *     "周期をまたぐ" 二値リセット。谷の中では毎周期 π で別々の局所解に
   *     飛びつき、検証されていない解がそのまま publish され続けていた。
   *
   *   新ロジック:
   *     plan() の1周期内で完結するリトライループ。
   *       1) delta = gimbal_delta_angle_ で出発点 x0 から最適化
   *       2) 不安定 or fc_t_min が小さすぎれば x を x0 に戻し、
   *          delta を gimbal_delta_escalation_factor_ 倍に広げて再試行
   *       3) 検証済みの安定解が見つかるか、最大リトライ回数
   *          (gimbal_delta_max_retries_) あるいは時間予算
   *          (gimbal_delta_max_time_) に達するまで繰り返す
   *       4) それでも見つからない場合のみ、最終手段として
   *          diagnoseGlobalMaxFCTMin() の結果を採用する
   * ========================================================================== */

  const std::vector<double> x0 = x; // この周期の出発点。各リトライで必ずここへ戻る
  const double loop_start_time = ros::Time::now().toSec();

  double delta_angle_used = gimbal_delta_angle_;
  bool   solve_ok = false;
  int    retry_count = 0;
  double achieved_fc_t_min = 0.0;
  double max_f = 0;

  std::vector<double> lb(x0.size()), ub(x0.size());

  if(first_run)
    {
      /* 初回はヒューリスティック初期値の近傍のみを1回だけ最適化する
       * （従来どおり）。エスカレーションは2周期目以降に限定する。 */
      for(size_t i = 0; i < x0.size(); i++)
        {
          lb.at(i) = -M_PI;
          ub.at(i) =  M_PI;
        }
      solver->set_lower_bounds(lb);
      solver->set_upper_bounds(ub);

      try { solver->optimize(x, max_f); }
      catch(std::exception &e) { ROS_WARN_STREAM("[navi][plan_debug] nlopt failed on first_run: " << e.what()); }

      applyGimbalAngles(this, x);
      solve_ok = robot_model_for_plan_->stabilityCheck(false);
      achieved_fc_t_min = robot_model_for_plan_->getFeasibleControlTMin();
      retry_count = 0;
    }
  else
    {
      for(retry_count = 0; retry_count <= gimbal_delta_max_retries_; retry_count++)
        {
          for(size_t i = 0; i < x0.size(); i++)
            {
              lb.at(i) = x0.at(i) - delta_angle_used;
              ub.at(i) = x0.at(i) + delta_angle_used;
            }
          solver->set_lower_bounds(lb);
          solver->set_upper_bounds(ub);

          x = x0; // ★ 毎リトライ、出発点から再スタート（前リトライの悪い解を引きずらない）

          try
            {
              solver->optimize(x, max_f);
            }
          catch(std::exception &e)
            {
              ROS_WARN_STREAM("[navi][plan_debug] nlopt failed at retry " << retry_count
                              << " (delta=" << delta_angle_used << "): " << e.what());
            }

          /* モデルを採用解 x の状態に戻してから判定する（COBYLA は棄却点で
           * 終わることがあるため） */
          applyGimbalAngles(this, x);
          bool stable = robot_model_for_plan_->stabilityCheck(false);
          achieved_fc_t_min = robot_model_for_plan_->getFeasibleControlTMin();

          if(stable && achieved_fc_t_min > gimbal_delta_fc_t_min_ok_)
            {
              solve_ok = true;
              break;
            }

          bool time_budget_exceeded =
            (ros::Time::now().toSec() - loop_start_time) > gimbal_delta_max_time_;

          if(retry_count < gimbal_delta_max_retries_ && !time_budget_exceeded)
            {
              double next_delta = std::min(delta_angle_used * gimbal_delta_escalation_factor_, M_PI);
              ROS_WARN_STREAM("[navi][plan_debug] escalation retry " << retry_count
                              << ": delta=" << delta_angle_used << " -> " << next_delta
                              << " (stable=" << stable << ", fc_t_min=" << achieved_fc_t_min << ")");
              delta_angle_used = next_delta;
            }
          else
            {
              if(time_budget_exceeded)
                ROS_WARN_STREAM("[navi][plan_debug] escalation time budget ("
                                << gimbal_delta_max_time_ << "s) exceeded at retry "
                                << retry_count << ", stopping early.");
              break;
            }
        }

      if(!solve_ok)
        {
          /* ★ 最終手段: リトライを使い切っても検証済みの解が見つからない
           *   場合のみ、重い広域グリッド探索を1回行い、その最良解を採用する。
           *   [rev.16] target_eval_budget=1728 は旧来のn=3,12分割相当の
           *   評価点数。n=4（gimbal1自由時）でも同程度のコストになるよう
           *   diagnoseGlobalMaxFCTMin内部で自動的にgrid_stepsを調整する。 */
          std::vector<double> diag_best_x;
          double global_max_fc_t_min = diagnoseGlobalMaxFCTMin(this, x0, 1728, diag_best_x);

          if(global_max_fc_t_min > gimbal_delta_fc_t_min_ok_)
            {
              ROS_ERROR_STREAM("[navi][plan_debug] escalation exhausted (" << retry_count
                               << " retries, final delta=" << delta_angle_used << "). "
                               << "falling back to global-search solution, fc_t_min="
                               << global_max_fc_t_min);
              x = diag_best_x;
              applyGimbalAngles(this, x);
              achieved_fc_t_min = global_max_fc_t_min;
            }
          else
            {
              ROS_ERROR_STREAM("[navi][plan_debug] TRUE SINGULARITY suspected: even global grid "
                               "search found max fc_t_min = " << global_max_fc_t_min
                               << " (this joint/gimbal1 configuration makes fc_t_min ~0 "
                               << "regardless of gimbal2,3,4 choice). using last escalation "
                               << "attempt (delta=" << delta_angle_used << ") as-is.");
              /* diag_best_x は使わず、最後のエスカレーション試行の x をそのまま使う。
               * （真の特異点近傍では広域探索の結果も信頼性が低いため。） */
            }
        }
    }

  /* ---- 以降の後処理（jump検出・publish用の値の確定） ---------------------- */
  double start_time = loop_start_time;
  try
    {
      /* 自由変数を全体ベクトルへ書き戻す */
      opt_gimbal_angles_ = composeGimbalAngles(x);

      /* ★ 固定ジンバル以外について、前周期解との最大跳躍量を計算 */
      last_max_jump_ = 0.0;
      if(prev_opt_gimbal_angles_.size() == opt_gimbal_angles_.size())
        {
          for(int i = 0; i < opt_gimbal_angles_.size(); i++)
            {
              if(active_fix_enabled_ && i == active_fix_idx_) continue; // 固定ジンバルはスルー制限別管理
              double d = fabs(normalizeAngle(opt_gimbal_angles_.at(i) - prev_opt_gimbal_angles_.at(i)));
              if(d > last_max_jump_) last_max_jump_ = d;
            }
          if(last_max_jump_ > gimbal_delta_angle_ * 1.5)
            {
              ROS_WARN_STREAM("[navi][plan_debug] LARGE GIMBAL JUMP detected: "
                              << last_max_jump_ << " rad (delta_angle_used=" << delta_angle_used
                              << ", retry_count=" << retry_count << ", solve_ok=" << solve_ok << ")");
            }
        }

      applyGimbalAngles(this, x);

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
          std::cout << ", retries: " << retry_count << ", solve_ok: " << solve_ok;
          std::cout << ", attitude: [" << roll << ", " << pitch;
          std::cout << "], force: [" << robot_model_for_plan_->getStaticThrust().transpose();
          std::cout << "]" << std::endl;
        }

      /* ★ この周期でのnlopt内部stabilityCheck失敗回数を保存 */
      last_invalid_cnt_ = invalid_cnt;
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
                    ? fabs(normalizeAngle(target - active_fix_angle_)) : M_PI;
  fix_gimbal_state_pub_.publish(state_msg);

  /* ★ [rev.12 変更] 診断情報のpublish。
   *   [0] : solve_ok        （この周期でエスカレーション込みで検証済みの解が見つかったか）
   *   [1] : delta_angle_used（最終的に使われた探索半幅）
   *   [2] : retry_count     （このplan()周期で要したリトライ回数。旧仕様の
   *                           invalid_cnt から意味が変わっているため、
   *                           Python側ログの文言更新を推奨）
   *   [3] : last_max_jump_
   *   [4]以降: opt_gimbal_angles_（実際のgimbal角）
   *   メッセージのフィールド数・順序は変更していないため、Python側の
   *   購読コード（_plan_debug_cb）は無変更で動作する。 */
  std_msgs::Float64MultiArray debug_msg;
  debug_msg.data.resize(4 + opt_gimbal_angles_.size());
  debug_msg.data[0] = solve_ok ? 1.0 : 0.0;
  debug_msg.data[1] = delta_angle_used;
  debug_msg.data[2] = static_cast<double>(retry_count);
  debug_msg.data[3] = last_max_jump_;
  for(size_t i = 0; i < opt_gimbal_angles_.size(); i++)
    debug_msg.data[4 + i] = opt_gimbal_angles_.at(i);
  plan_debug_pub_.publish(debug_msg);

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

  /* ★ [rev.12 追加] 探索範囲エスカレーション用パラメータ */
  getParam<double>(navi_nh, "gimbal_delta_escalation_factor", gimbal_delta_escalation_factor_, 2.0);
  getParam<int>(navi_nh, "gimbal_delta_max_retries", gimbal_delta_max_retries_, 5);
  getParam<double>(navi_nh, "gimbal_delta_fc_t_min_ok", gimbal_delta_fc_t_min_ok_, 0.05);
  getParam<double>(navi_nh, "gimbal_delta_max_time", gimbal_delta_max_time_, 0.03); // [s] 20Hz(50ms)周期に対する安全弁

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