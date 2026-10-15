#include "neutral_flexion_axis_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace neutral_axis_controllers {

namespace {
Vector6d make6(double trans, double rot) {
  Vector6d v;
  v << trans, trans, trans, rot, rot, rot;
  return v;
}

// Weiche Totzone: Vektoren kleiner als d werden 0, größere um d verkürzt (stetig)
Eigen::Vector3d deadband(const Eigen::Vector3d& v, double d) {
  const double n = v.norm();
  if (n <= d) return Eigen::Vector3d::Zero();
  return v * (1.0 - d / n);
}
}  // namespace

// ============================================================================
// Lifecycle
// ============================================================================

controller_interface::CallbackReturn NeutralFlexionAxisController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("nullspace_damping", 2.0);
    auto_declare<double>("max_force", 40.0);
    auto_declare<double>("max_torque", 8.0);

    // Impedanzregler (innere Schleife)
    auto_declare<double>("impedance_trans_stiffness", 200.0);  // N/m
    auto_declare<double>("impedance_rot_stiffness", 20.0);     // Nm/rad
    auto_declare<double>("free_trans_damping", 15.0);          // Ns/m
    auto_declare<double>("free_rot_damping", 1.0);             // Nms/rad

    // Admittanz (äußere Schleife)
    auto_declare<double>("admittance_mass", 1.0);              // kg
    auto_declare<double>("admittance_rot_mass", 0.05);         // kgm^2
    auto_declare<double>("admittance_max_lin_vel", 0.1);       // m/s
    auto_declare<double>("admittance_max_ang_vel", 0.3);       // rad/s
    auto_declare<int>("outer_loop_divider", 10);

    // Führung um die Knieachse (Startwerte aus OptiTrack, Basis-Frame!)
    auto_declare<std::vector<double>>("knee_center", std::vector<double>{0.0, 0.0, 0.0});  // m
    auto_declare<std::vector<double>>("knee_axis", std::vector<double>{0.0, 1.0, 0.0});
    auto_declare<double>("guidance_direction", 1.0);           // +1/-1: Drehsinn um knee_axis = Extension
    auto_declare<double>("guidance_ang_vel_deg", 3.0);         // Grad/s Knie-Winkelgeschwindigkeit
    auto_declare<double>("guidance_ramp_time", 5.0);           // s
    auto_declare<double>("guidance_lin_damping_along", 50.0);  // Ns/m entlang Tangente (niedrig)
    auto_declare<double>("guidance_lin_damping_across", 150.0);// Ns/m quer dazu (hoch)
    auto_declare<double>("guidance_rot_damping_about", 1.0);   // Nms/rad um die Knieachse (niedrig)
    auto_declare<double>("guidance_rot_damping_across", 5.0);  // Nms/rad um die anderen Achsen
    auto_declare<double>("max_guidance_force", 10.0);          // N entlang Tangente -> ERROR
    auto_declare<double>("max_interaction_force", 30.0);       // N |F_interaction| -> ERROR
    auto_declare<double>("max_sweep_angle_deg", 115.0);        // Grad, harte Winkelgrenze -> FINISHED
    auto_declare<bool>("use_optitrack_progress", false);       // Kniewinkel-Topic als Fortschritt
    auto_declare<double>("target_knee_angle_deg", 5.0);        // Grad -> FINISHED
    auto_declare<double>("knee_angle_timeout", 0.5);           // s

    // Anregung (vorerst AUS: reine geführte Bewegung zur Achsbestimmung)
    auto_declare<double>("exploration_force_amp", 0.0);        // N
    auto_declare<double>("exploration_torque_amp", 0.0);       // Nm
    auto_declare<double>("exploration_ramp_time", 5.0);        // s
    auto_declare<double>("exploration_duration", 60.0);        // s

    // Ablauf / Sicherheit
    auto_declare<double>("calibration_duration", 5.0);         // s
    auto_declare<bool>("estimate_leg", true);                  // false: nur F/T-Offset (Test ohne Bein)
    auto_declare<double>("interaction_deadband_force", 0.0);   // N
    auto_declare<double>("interaction_deadband_torque", 0.0);  // Nm
    auto_declare<double>("max_tracking_error", 0.10);          // m
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  nullspace_damping_ = node->get_parameter("nullspace_damping").as_double();
  max_force_ = node->get_parameter("max_force").as_double();
  max_torque_ = node->get_parameter("max_torque").as_double();
  calibration_duration_ = node->get_parameter("calibration_duration").as_double();
  exploration_duration_ = node->get_parameter("exploration_duration").as_double();
  max_tracking_error_ = node->get_parameter("max_tracking_error").as_double();
  outer_loop_divider_ = std::max<int>(1, node->get_parameter("outer_loop_divider").as_int());

  // Impedanzregler
  Matrix6d K = Matrix6d::Zero();
  Matrix6d D = Matrix6d::Zero();
  K.diagonal() = make6(node->get_parameter("impedance_trans_stiffness").as_double(),
                       node->get_parameter("impedance_rot_stiffness").as_double());
  D.diagonal() = make6(node->get_parameter("free_trans_damping").as_double(),
                       node->get_parameter("free_rot_damping").as_double());
  impedance_controller_.configure(K, D);

  // Admittanz (Dämpfung und v_ref kommen zur Laufzeit aus guidance_)
  admittance_.configure(make6(node->get_parameter("admittance_mass").as_double(),
                              node->get_parameter("admittance_rot_mass").as_double()),
                        node->get_parameter("admittance_max_lin_vel").as_double(),
                        node->get_parameter("admittance_max_ang_vel").as_double());

  // Führung um die Knieachse
  const auto c = node->get_parameter("knee_center").as_double_array();
  const auto a = node->get_parameter("knee_axis").as_double_array();
  if (c.size() != 3 || a.size() != 3 ||
      Eigen::Vector3d(a[0], a[1], a[2]).norm() < 1e-6) {
    RCLCPP_ERROR(node->get_logger(), "knee_center/knee_axis ungültig (je 3 Werte, Achse != 0)");
    return controller_interface::CallbackReturn::ERROR;
  }
  guidance_.configure(
      Eigen::Vector3d(c[0], c[1], c[2]), Eigen::Vector3d(a[0], a[1], a[2]),
      node->get_parameter("guidance_direction").as_double() >= 0.0 ? 1.0 : -1.0,
      node->get_parameter("guidance_ang_vel_deg").as_double() * M_PI / 180.0,
      node->get_parameter("guidance_lin_damping_along").as_double(),
      node->get_parameter("guidance_lin_damping_across").as_double(),
      node->get_parameter("guidance_rot_damping_about").as_double(),
      node->get_parameter("guidance_rot_damping_across").as_double(),
      node->get_parameter("guidance_ramp_time").as_double());
  max_guidance_force_ = node->get_parameter("max_guidance_force").as_double();
  max_interaction_force_ = node->get_parameter("max_interaction_force").as_double();
  max_sweep_angle_ = node->get_parameter("max_sweep_angle_deg").as_double() * M_PI / 180.0;
  use_optitrack_progress_ = node->get_parameter("use_optitrack_progress").as_bool();
  target_knee_angle_ = node->get_parameter("target_knee_angle_deg").as_double() * M_PI / 180.0;
  knee_angle_timeout_ = node->get_parameter("knee_angle_timeout").as_double();
  estimate_leg_ = node->get_parameter("estimate_leg").as_bool();
  deadband_force_ = node->get_parameter("interaction_deadband_force").as_double();
  deadband_torque_ = node->get_parameter("interaction_deadband_torque").as_double();

  // Kniewinkel (OptiTrack) in GRAD, 0 = gestreckt
  knee_angle_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      "~/knee_angle", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Float64::SharedPtr msg) {
        KneeAngleSample smp;
        smp.angle_rad = msg->data * M_PI / 180.0;
        smp.stamp = get_node()->now().seconds();
        knee_angle_buffer_.writeFromNonRT(smp);
      });

  // Debug-Publisher für die Auswertung
  guidance_pub_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>("~/guidance_recorded", 10);
  rt_guidance_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
          guidance_pub_);
  rt_guidance_pub_->msg_.data.assign(27, 0.0);

  // Anregung: verschiedene, inkommensurable Frequenzen pro Achse (Hz)
  Vector6d freq;
  freq << 0.10, 0.17, 0.23, 0.31, 0.41, 0.53;
  exploration_generator_.configure(
      make6(node->get_parameter("exploration_force_amp").as_double(),
            node->get_parameter("exploration_torque_amp").as_double()),
      freq, node->get_parameter("exploration_ramp_time").as_double());

  // Franka Robot Model (Jacobian, Pose, Coriolis)
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                     arm_id_ + "/robot_state"));

  for (const auto& name : franka_robot_state_->get_state_interface_names()) {
    RCLCPP_INFO(node->get_logger(), "FrankaRobotState Interface: %s", name.c_str());
  }

  // Start der Exploration (nur wirksam im Zustand READY):
  //   ros2 service call /<controller>/start_exploration std_srvs/srv/Trigger
  start_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/start_exploration",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        start_requested_.store(true);
        res->success = true;
        res->message = "Start angefordert (wirkt nur im Zustand READY)";
      });

  // Logging-Publisher (Realtime-safe, siehe INTEGRATION.md für ros2 bag record)
  joint_state_pub_ =
      node->create_publisher<sensor_msgs::msg::JointState>("~/joint_states_recorded", 10);
  rt_joint_state_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
          joint_state_pub_);
  rt_joint_state_pub_->msg_.name.resize(kNumJoints);
  rt_joint_state_pub_->msg_.position.resize(kNumJoints);
  rt_joint_state_pub_->msg_.velocity.resize(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    rt_joint_state_pub_->msg_.name[i] = arm_id_ + "_joint" + std::to_string(i + 1);
  }

  wrench_pub_ =
      node->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench_ext_recorded", 10);
  rt_wrench_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
          wrench_pub_);
  rt_wrench_pub_->msg_.header.frame_id = arm_id_ + "_K";  // Franka "K"/Stiffness-Frame ~ EE

  return controller_interface::CallbackReturn::SUCCESS;
}

// commanded by controller: Effort/torque for every joint
controller_interface::InterfaceConfiguration
NeutralFlexionAxisController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// needed states: joint positions + velocities, robot model, F/T estimate
// ACHTUNG: Reihenfolge ist wichtig (readRobotState indexiert 2*i und 2*i+1)
controller_interface::InterfaceConfiguration
NeutralFlexionAxisController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  auto model_interfaces = franka_robot_model_->get_state_interface_names();
  config.names.insert(config.names.end(), model_interfaces.begin(), model_interfaces.end());
  // Das Robot-State-Interface (liefert u.a. K_F_ext_hat_K). Normalerweise fordert FrankaRobotModel
  // es schon an -> nicht doppelt eintragen. Gelenk-Interfaces bleiben vorn (2*i / 2*i+1).
  const std::string state_name = arm_id_ + "/robot_state";
  if (std::find(config.names.begin(), config.names.end(), state_name) == config.names.end()) {
    config.names.push_back(state_name);
  }
  return config;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  franka_state_ = nullptr;  // wird im ersten update() aufgelöst

  // Zustand zurücksetzen: Start immer mit Kalibrierung
  leg_mass_ = 0.0;
  leg_com_k_.setZero();
  cal_aa_ = cal_af_ = 0.0;
  cal_AtA_.setZero();
  cal_Atb_.setZero();
  cal_samples_ = 0;
  cal_sum_f_.setZero();
  cal_sum_tau_.setZero();
  bias_f_.setZero();
  bias_tau_.setZero();
  start_requested_.store(false);
  knee_angle_buffer_.writeFromNonRT(KneeAngleSample{});  // stamp = alt -> stale bis Daten kommen
  state_ = ExperimentState::CALIBRATION;
  state_time_ = 0.0;
  cycle_counter_ = 0;
  outer_dt_acc_ = 0.0;
  // x_hold_ wird im ersten update()-Zyklus gesetzt (cycle_counter_ == 0 in CALIBRATION)
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  franka_state_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Hilfsfunktionen
// ============================================================================

Wrench NeutralFlexionAxisController::saturateWrench(const Wrench& F) const {
  Wrench out = F;
  const double f_norm = out.head<3>().norm();
  if (f_norm > max_force_) {
    out.head<3>() *= (max_force_ / f_norm);
  }
  const double tau_norm = out.tail<3>().norm();
  if (tau_norm > max_torque_) {
    out.tail<3>() *= (max_torque_ / tau_norm);
  }
  return out;
}

void NeutralFlexionAxisController::readRobotState(
    Eigen::Matrix<double, kNumJoints, 1>& q, Eigen::Matrix<double, kNumJoints, 1>& dq) const {
  for (int i = 0; i < kNumJoints; ++i) {
    q(i) = state_interfaces_[2 * i].get_value();
    dq(i) = state_interfaces_[2 * i + 1].get_value();
  }
}

// externe Kraft/Moment im K-("EE"-)Frame, wie von franka_hardware geschätzt
// Pointer auf franka::RobotState aus dem Interface "<arm_id>/robot_state" holen.
// Das Interface speichert die Bits des Pointers in einem double (Franka-Konvention,
// siehe bit_cast in franka_robot_state.cpp).
bool NeutralFlexionAxisController::resolveFrankaState() {
  if (franka_state_ != nullptr) return true;
  const std::string name = arm_id_ + "/robot_state";
  for (const auto& si : state_interfaces_) {
    if (si.get_name() == name) {
      const double v = si.get_value();
      const franka::RobotState* ptr = nullptr;
      static_assert(sizeof(ptr) == sizeof(v), "Pointer und double müssen gleich groß sein");
      std::memcpy(&ptr, &v, sizeof(ptr));
      franka_state_ = ptr;
      return franka_state_ != nullptr;
    }
  }
  return false;
}

// K_F_ext_hat_K: [fx fy fz tx ty tz] im Stiffness-Frame K, direkt aus libfranka (keine Kopie)
bool NeutralFlexionAxisController::readFTSensor(Eigen::Vector3d& f_ext_k,
                                                Eigen::Vector3d& tau_ext_k) {
  if (!resolveFrankaState()) return false;
  const auto& w = franka_state_->K_F_ext_hat_K;
  f_ext_k << w[0], w[1], w[2];
  tau_ext_k << w[3], w[4], w[5];
  return true;
}

// Kraft/Moment, die das Bein (Gewicht) auf den Roboter ausübt, im Basis-Frame.
// Modell: m, Schwerpunkt c (im K-Frame) -> f = [0 0 -m*g], tau = (R*c) x f
Wrench NeutralFlexionAxisController::legGravityWrench(const Eigen::Matrix3d& R) const {
  Wrench w = Wrench::Zero();
  const Eigen::Vector3d f(0.0, 0.0, -leg_mass_ * kGravity);
  w.head<3>() = f;
  w.tail<3>() = (R * leg_com_k_).cross(f);
  return w;
}

// tau = J^T F_cmd  +  Coriolis  +  Nullraumdämpfung  -  J^T F_g_leg (Beingewicht kompensieren)
// (Robotergewicht kompensiert die Franka-Hardware selbst)
Eigen::Matrix<double, NeutralFlexionAxisController::kNumJoints, 1>
NeutralFlexionAxisController::computeTorque(const ControlInput& in, const Wrench& F_cmd) const {
  // Gedämpfte Pseudoinverse (7 Gelenke, 6 kartesische Zwänge -> 1 Redundanz)
  const Eigen::Matrix<double, 6, 6> JJt =
      in.J * in.J.transpose() + 1e-4 * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 7, 6> J_pinv = in.J.transpose() * JJt.inverse();
  const Eigen::Matrix<double, 7, 7> N =
      Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * in.J;
  const Eigen::Matrix<double, 7, 1> tau_null = N.transpose() * (-nullspace_damping_ * in.dq);

  // Das Bein drückt mit F_g_leg auf den Roboter -> Roboter muss -F_g_leg zusätzlich aufbringen
  const Wrench F_total = F_cmd - in.F_g_leg;
  return in.J.transpose() * F_total + in.coriolis + tau_null;
}

void NeutralFlexionAxisController::enterState(ExperimentState s, const Pose& x) {
  state_ = s;
  state_time_ = 0.0;
  cycle_counter_ = 0;
  outer_dt_acc_ = 0.0;
  x_hold_ = x;  // aktuelle Pose als Haltepose
  if (s == ExperimentState::EXPLORATION) {
    admittance_.initialize(x);
    guidance_.start(x.p);
    F_explore_.setZero();
    F_interaction_.setZero();
  }
}

// ============================================================================
// Zustände
// ============================================================================

// Kalibrierung: Roboter hält die Pose, Beingewicht (m, Schwerpunkt c) wird per Least Squares
// aus statischen Samples geschätzt:
//   f_k   = m * a_k,  a_k = -g * R^T e_z        -> m = sum(a.f)/sum(a.a)
//   tau_k = c x f_k = -[f_k]x * c               -> c aus Normalgleichung (min. 2 verschiedene
//                                                  Orientierungen nötig, sonst ist c entlang f
//                                                  nicht beobachtbar)
// TODO: c mit OptiTrack-Knieposition verknüpfen
Wrench NeutralFlexionAxisController::updateCalibration(const ControlInput& in,
                                                       const Eigen::Vector3d& f_ext_k,
                                                       const Eigen::Vector3d& tau_ext_k) {
  // Nur statische Samples verwenden
  if (in.dq.norm() < 0.02) {
    const Eigen::Vector3d a = -kGravity * in.R.transpose() * Eigen::Vector3d::UnitZ();
    cal_aa_ += a.dot(a);
    cal_af_ += a.dot(f_ext_k);

    Eigen::Matrix3d skew;
    skew << 0.0, -f_ext_k.z(), f_ext_k.y(),
            f_ext_k.z(), 0.0, -f_ext_k.x(),
            -f_ext_k.y(), f_ext_k.x(), 0.0;
    const Eigen::Matrix3d A = -skew;  // tau = A * c
    cal_AtA_ += A.transpose() * A;
    cal_Atb_ += A.transpose() * tau_ext_k;
    cal_sum_f_ += f_ext_k;
    cal_sum_tau_ += tau_ext_k;
    ++cal_samples_;
  }

  if (state_time_ >= calibration_duration_) {
    if (finishCalibration()) {
      enterState(ExperimentState::READY, in.x);
    } else {
      enterState(ExperimentState::ERROR, in.x);
    }
  }
  return impedance_controller_.compute_wrench(x_hold_, Vector6d::Zero(), in.x, in.dx);
}

bool NeutralFlexionAxisController::finishCalibration() {
  if (cal_samples_ < 100 || cal_aa_ < 1e-6) {
    RCLCPP_ERROR(get_node()->get_logger(), "Kalibrierung: zu wenige statische Samples (%d)",
                 cal_samples_);
    return false;
  }
  if (!estimate_leg_) {
    // Test ohne Bein: nur den statischen F/T-Offset bestimmen, Beinmodell = 0
    bias_f_ = cal_sum_f_ / cal_samples_;
    bias_tau_ = cal_sum_tau_ / cal_samples_;
    leg_mass_ = 0.0;
    leg_com_k_.setZero();
    RCLCPP_INFO(get_node()->get_logger(),
                "Offset-Kalibrierung: f=[%.2f %.2f %.2f] N, tau=[%.3f %.3f %.3f] Nm",
                bias_f_.x(), bias_f_.y(), bias_f_.z(), bias_tau_.x(), bias_tau_.y(),
                bias_tau_.z());
    return true;
  }
  leg_mass_ = std::max(0.0, cal_af_ / cal_aa_);

  // Tikhonov-regularisiert, damit bei nur einer Orientierung nichts explodiert
  const Eigen::Matrix3d AtA = cal_AtA_ / cal_samples_;
  const Eigen::Vector3d Atb = cal_Atb_ / cal_samples_;
  leg_com_k_ = (AtA + 1e-3 * Eigen::Matrix3d::Identity()).ldlt().solve(Atb);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(AtA);
  if (es.eigenvalues().minCoeff() < 1.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Kalibrierung: Schwerpunkt nur schwach beobachtbar (zu wenig Orientierungsvielfalt)");
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "Kalibrierung fertig: m=%.2f kg, c_K=[%.3f %.3f %.3f] m", leg_mass_,
              leg_com_k_.x(), leg_com_k_.y(), leg_com_k_.z());
  return true;
}

Wrench NeutralFlexionAxisController::updateReady(const ControlInput& in) {
  if (start_requested_.exchange(false)) {
    enterState(ExperimentState::EXPLORATION, in.x);
  }
  return impedance_controller_.compute_wrench(x_hold_, Vector6d::Zero(), in.x, in.dx);
}

// MOST IMPORTANT FUNCTION (ACTUAL CONTROLLER):
// outer loop (lower freq): Führung (Tangente, anisotrope Dämpfung, v_ref aus aktueller Pose)
//   F_interaction = F_ext - F_g_leg
//   M ddx_d + D (dx_d - v_ref) = F_explore + F_interaction -> x_d, dx_d
// inner loop (every cycle): x_d, dx_d -> cartesian impedance controller
//
// Hinweis: F_ext_hat (Franka) enthält NICHT die eigene Anregungskraft und nicht die
// Robotergewichtskraft. Die Bias-Kraft der Führung ist ~ D_along * v_lin (klein halten).
Wrench NeutralFlexionAxisController::updateExploration(const ControlInput& in) {
  outer_dt_acc_ += in.dt;
  if (cycle_counter_ % outer_loop_divider_ == 0) {
    F_explore_ = exploration_generator_.excite(state_time_);
    F_interaction_ = in.F_ext_base - in.F_g_leg;
    F_interaction_.head<3>() = deadband(F_interaction_.head<3>(), deadband_force_);
    F_interaction_.tail<3>() = deadband(F_interaction_.tail<3>(), deadband_torque_);

    // Richtung (Tangente), anisotrope Dämpfung und v_ref JEDEN Außenschritt neu
    last_guidance_ = guidance_.compute(in.x.p, state_time_);
    admittance_.setGuidance(last_guidance_.D, last_guidance_.v_ref);
    admittance_.update(F_explore_, F_interaction_, outer_dt_acc_);
    outer_dt_acc_ = 0.0;

    // Kraftgrenzen (Anschlag / Schmerzgrenze / Blockade)
    const Eigen::Vector3d f_int = F_interaction_.head<3>();
    if (f_int.norm() > max_interaction_force_ ||
        std::abs(f_int.dot(last_guidance_.t)) > max_guidance_force_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interaktionskraft zu groß (|F|=%.1f N) -> ERROR",
                   f_int.norm());
      enterState(ExperimentState::ERROR, in.x);
      return applySafeCommand(in);
    }
  }
  ++cycle_counter_;

  // Sicherheit: Solltrajektorie darf nicht weit von der Ist-Pose weglaufen (Windup)
  if ((admittance_.pose().p - in.x.p).norm() > max_tracking_error_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Tracking-Fehler zu groß -> ERROR");
    enterState(ExperimentState::ERROR, in.x);
    return applySafeCommand(in);
  }

  // Fortschritt / Abbruch
  last_sweep_ = guidance_.sweepAngle(in.x.p);
  if (use_optitrack_progress_) {
    const KneeAngleSample smp = *knee_angle_buffer_.readFromRT();
    if (in.t_now - smp.stamp > knee_angle_timeout_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Kniewinkel-Daten veraltet -> ERROR");
      enterState(ExperimentState::ERROR, in.x);
      return applySafeCommand(in);
    }
    last_knee_angle_ = smp.angle_rad;
    if (smp.angle_rad <= target_knee_angle_) {
      enterState(ExperimentState::FINISHED, in.x);
      return holdSafePosition(in);
    }
  }
  if (last_sweep_ >= max_sweep_angle_ || state_time_ > exploration_duration_) {
    enterState(ExperimentState::FINISHED, in.x);
    return holdSafePosition(in);
  }

  return impedance_controller_.compute_wrench(admittance_.pose(), admittance_.velocity(),
                                              in.x, in.dx);
}

// Pose halten, Beingewicht wird in computeTorque() kompensiert
Wrench NeutralFlexionAxisController::holdSafePosition(const ControlInput& in) {
  return impedance_controller_.compute_wrench(x_hold_, Vector6d::Zero(), in.x, in.dx);
}

// Nur dämpfen (x_d = x -> Federanteil 0), Beingewicht wird weiter kompensiert
Wrench NeutralFlexionAxisController::applySafeCommand(const ControlInput& in) {
  return impedance_controller_.compute_wrench(in.x, Vector6d::Zero(), in.x, in.dx);
}

// ============================================================================
// Update
// ============================================================================

controller_interface::return_type NeutralFlexionAxisController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  ControlInput in;
  in.dt = period.seconds();
  in.t_now = time.seconds();

  // --- State + F/T --------------------------------------------------------
  Eigen::Vector3d f_ext_k, tau_ext_k;
  readRobotState(in.q, in.dq);
  if (!readFTSensor(f_ext_k, tau_ext_k)) {
    return controller_interface::return_type::ERROR;  // kein Kraftsignal -> Controller stoppt
  }

  // --- Kinematik / Dynamik vom Franka Robot Model --------------------------
  const std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  in.J = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());

  const std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  in.coriolis = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

  const std::array<double, 16> pose_array =
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  const Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(pose_array.data());
  in.R = T.topLeftCorner<3, 3>();
  in.x.p = T.topRightCorner<3, 1>();
  in.x.q = Eigen::Quaterniond(in.R).normalized();

  in.dx = in.J * in.dq;  // Basis-Frame

  // --- Externe Kraft ins Basis-Frame, Beingewicht aus Modell ----------------
  in.F_ext_base.head<3>() = in.R * (f_ext_k - bias_f_);
  in.F_ext_base.tail<3>() = in.R * (tau_ext_k - bias_tau_);
  in.F_g_leg = legGravityWrench(in.R);

  // Erster Zyklus: Haltepose setzen
  if (state_ == ExperimentState::CALIBRATION && cycle_counter_ == 0 && state_time_ == 0.0) {
    x_hold_ = in.x;
  }

  state_time_ += in.dt;

  // --- State machine --------------------------------------------------------
  Wrench F_cmd = Wrench::Zero();
  switch (state_) {
    case ExperimentState::CALIBRATION:
      F_cmd = updateCalibration(in, f_ext_k, tau_ext_k);
      break;
    case ExperimentState::READY:
      F_cmd = updateReady(in);
      break;
    case ExperimentState::EXPLORATION:
      F_cmd = updateExploration(in);
      break;
    case ExperimentState::FINISHED:
      F_cmd = holdSafePosition(in);
      break;
    case ExperimentState::ERROR:
      F_cmd = applySafeCommand(in);
      break;
  }

  // --- Drehmomentkommando -----------------------------------------------------
  const Eigen::Matrix<double, kNumJoints, 1> tau_cmd = computeTorque(in, saturateWrench(F_cmd));
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  // --- Logging: q, dq, F_ext/tau_ext für Post-Processing --------------
  if (rt_joint_state_pub_->trylock()) {
    rt_joint_state_pub_->msg_.header.stamp = time;
    for (int i = 0; i < kNumJoints; ++i) {
      rt_joint_state_pub_->msg_.position[i] = in.q(i);
      rt_joint_state_pub_->msg_.velocity[i] = in.dq(i);
    }
    rt_joint_state_pub_->unlockAndPublish();
  }
  if (rt_wrench_pub_->trylock()) {
    rt_wrench_pub_->msg_.header.stamp = time;
    rt_wrench_pub_->msg_.wrench.force.x = f_ext_k.x();
    rt_wrench_pub_->msg_.wrench.force.y = f_ext_k.y();
    rt_wrench_pub_->msg_.wrench.force.z = f_ext_k.z();
    rt_wrench_pub_->msg_.wrench.torque.x = tau_ext_k.x();
    rt_wrench_pub_->msg_.wrench.torque.y = tau_ext_k.y();
    rt_wrench_pub_->msg_.wrench.torque.z = tau_ext_k.z();
    rt_wrench_pub_->unlockAndPublish();
  }
  if (rt_guidance_pub_->trylock()) {
    auto& d = rt_guidance_pub_->msg_.data;
    d[0] = static_cast<double>(state_);
    d[1] = last_sweep_;
    d[2] = last_knee_angle_;
    for (int i = 0; i < 6; ++i) {
      d[3 + i] = F_interaction_(i);
      d[15 + i] = last_guidance_.v_ref(i);
      d[21 + i] = in.F_g_leg(i);
    }
    for (int i = 0; i < 3; ++i) {
      d[9 + i] = admittance_.pose().p(i);
      d[12 + i] = in.x.p(i);
    }
    rt_guidance_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace neutral_axis_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(neutral_axis_controllers::NeutralFlexionAxisController,
                       controller_interface::ControllerInterface)