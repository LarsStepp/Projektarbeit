#include "neutral_flexion_axis_controller.hpp"

#include <cmath>

namespace neutral_axis_controllers {

controller_interface::CallbackReturn NeutralFlexionAxisController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<int>("driven_axis", 5);
    auto_declare<double>("axis_velocity", 0.05);
    auto_declare<double>("driven_axis_damping", 200.0);
    auto_declare<double>("free_trans_damping", 15.0);
    auto_declare<double>("free_rot_damping", 1.0);
    auto_declare<double>("nullspace_damping", 2.0);
    auto_declare<double>("max_force", 40.0);
    auto_declare<double>("max_torque", 8.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  driven_axis_ = get_node()->get_parameter("driven_axis").as_int();
  axis_velocity_ = get_node()->get_parameter("axis_velocity").as_double();
  driven_axis_damping_ = get_node()->get_parameter("driven_axis_damping").as_double();
  free_trans_damping_ = get_node()->get_parameter("free_trans_damping").as_double();
  free_rot_damping_ = get_node()->get_parameter("free_rot_damping").as_double();
  nullspace_damping_ = get_node()->get_parameter("nullspace_damping").as_double();
  max_force_ = get_node()->get_parameter("max_force").as_double();
  max_torque_ = get_node()->get_parameter("max_torque").as_double();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                     arm_id_ + "/robot_state"));

  // K_F_ext_hat_K wird seit franka_ros2 v2.x als ForceTorqueSensor state interface
  // exportiert: "<arm_id>_K_F_ext_hat_K/force.x", ".../force.y", ... ".../torque.z"
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      arm_id_ + "_K_F_ext_hat_K");

  // Logging-Publisher (Realtime-safe, siehe INTEGRATION.md für ros2 bag record)
  joint_state_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>("~/joint_states_recorded", 10);
  rt_joint_state_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
          joint_state_pub_);
  rt_joint_state_pub_->msg_.name.resize(kNumJoints);
  rt_joint_state_pub_->msg_.position.resize(kNumJoints);
  rt_joint_state_pub_->msg_.velocity.resize(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    rt_joint_state_pub_->msg_.name[i] = arm_id_ + "_joint" + std::to_string(i + 1);
  }

  wrench_pub_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_ext_recorded", 10);
  rt_wrench_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
          wrench_pub_);
  rt_wrench_pub_->msg_.header.frame_id = arm_id_ + "_K";  // Franka "K"/Stiffness-Frame ~ EE

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
NeutralFlexionAxisController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

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
  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  config.names.insert(config.names.end(), ft_interfaces.begin(), ft_interfaces.end());
  return config;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  force_torque_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

Eigen::Matrix<double, 6, 1> NeutralFlexionAxisController::buildAxisDirectionInBase(
    const Eigen::Matrix3d& R_ee_in_base) const {
  Eigen::Matrix<double, 6, 1> dir = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d e = Eigen::Vector3d::Zero();
  int local_idx = driven_axis_ % 3;
  e(local_idx) = 1.0;
  Eigen::Vector3d dir_base = R_ee_in_base * e;  // EE-Achsrichtung, ausgedrückt in Basis-Frame
  if (driven_axis_ < 3) {
    dir.head<3>() = dir_base;   // translatorisch
  } else {
    dir.tail<3>() = dir_base;   // rotatorisch
  }
  return dir;
}

Eigen::Matrix<double, 6, 1> NeutralFlexionAxisController::saturateWrench(
    const Eigen::Matrix<double, 6, 1>& F) const {
  Eigen::Matrix<double, 6, 1> out = F;
  double f_norm = out.head<3>().norm();
  if (f_norm > max_force_) {
    out.head<3>() *= (max_force_ / f_norm);
  }
  double tau_norm = out.tail<3>().norm();
  if (tau_norm > max_torque_) {
    out.tail<3>() *= (max_torque_ / tau_norm);
  }
  return out;
}

controller_interface::return_type NeutralFlexionAxisController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  // --- Zustand lesen ------------------------------------------------
  Eigen::Matrix<double, kNumJoints, 1> q, dq;
  for (int i = 0; i < kNumJoints; ++i) {
    q(i) = state_interfaces_[2 * i].get_value();
    dq(i) = state_interfaces_[2 * i + 1].get_value();
  }

  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix4d> T_ee_in_base(pose_array.data());
  Eigen::Matrix3d R_ee_in_base = T_ee_in_base.block<3, 3>(0, 0);

  // externe Kraft/Moment im K-("EE"-)Frame, wie von franka_hardware geschätzt
  Eigen::Vector3d f_ext_k(force_torque_sensor_->get_forces()[0],
                          force_torque_sensor_->get_forces()[1],
                          force_torque_sensor_->get_forces()[2]);
  Eigen::Vector3d tau_ext_k(force_torque_sensor_->get_torques()[0],
                             force_torque_sensor_->get_torques()[1],
                             force_torque_sensor_->get_torques()[2]);

  // --- Kartesische Geschwindigkeit -----------------------------------
  Eigen::Matrix<double, 6, 1> v_ee_base = J * dq;  // in Basis-Frame

  // --- Achsauswahl: Richtung der geführten Achse, ausgedrückt in Basis-Frame
  Eigen::Matrix<double, 6, 1> axis_dir = buildAxisDirectionInBase(R_ee_in_base);  // Einheitsvektor
  Eigen::Matrix<double, 6, 6> S = axis_dir * axis_dir.transpose();                // Projektor: geführte Achse
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity() - S;    // Projektor: freie Achsen

  // --- Regelgesetz -----------------------------------------------------
  // 1) geführte Achse: Geschwindigkeitsfehler-Rückführung (quasi-kinematisch)
  Eigen::Matrix<double, 6, 1> v_ref = axis_velocity_ * axis_dir;
  Eigen::Matrix<double, 6, 1> F_driven = S * (driven_axis_damping_ * (v_ref - v_ee_base));

  // 2) freie Achsen: reiner virtueller Dämpfer, K -> 0 (Admittanzverhalten,
  //    die externe Kraft/das Moment des Knies bewegt den EE frei gegen die Dämpfung)
  Eigen::Matrix<double, 6, 6> D_free = Eigen::Matrix<double, 6, 6>::Zero();
  D_free.diagonal() << free_trans_damping_, free_trans_damping_, free_trans_damping_,
      free_rot_damping_, free_rot_damping_, free_rot_damping_;
  Eigen::Matrix<double, 6, 1> F_free = P * (-D_free * v_ee_base);

  Eigen::Matrix<double, 6, 1> F_cmd = saturateWrench(F_driven + F_free);

  // --- Nullraum-Dämpfung (7 Gelenke, 6 kartesische Zwänge -> 1 Redundanz) ---
  Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * (J * J.transpose()).inverse();
  Eigen::Matrix<double, 7, 7> N =
      Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;
  Eigen::Matrix<double, 7, 1> tau_null = N.transpose() * (-nullspace_damping_ * dq);

  // --- Drehmomentkommando (Gravitation kompensiert libfranka intern selbst!) --
  Eigen::Matrix<double, 7, 1> tau_cmd = J.transpose() * F_cmd + coriolis + tau_null;

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  // --- Logging: q, dq, F_ext/tau_ext für Post-Processing --------------
  if (rt_joint_state_pub_->trylock()) {
    rt_joint_state_pub_->msg_.header.stamp = time;
    for (int i = 0; i < kNumJoints; ++i) {
      rt_joint_state_pub_->msg_.position[i] = q(i);
      rt_joint_state_pub_->msg_.velocity[i] = dq(i);
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

  return controller_interface::return_type::OK;
}

}  // namespace neutral_axis_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(neutral_axis_controllers::NeutralFlexionAxisController,
                        controller_interface::ControllerInterface)
