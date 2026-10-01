#pragma once

#include "neutral_flexion_axis_controller.hpp"
#include "admittance_system.hpp"
#include "impedance_controller.hpp"
#include <cmath>

namespace neutral_axis_controllers {

controller_interface::CallbackReturn NeutralFlexionAxisController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("free_trans_damping", 15.0);
    auto_declare<double>("free_rot_damping", 1.0);
    auto_declare<double>("nullspace_damping", 2.0);
    auto_declare<double>("max_force", 40.0);
    auto_declare<double>("max_torque", 8.0);
    auto_declare<double>("admittance_mass",1.0);
    auto_declare<double>("admittance_damping, 20.0");

    admittance_ = AdmittanceSystem();
    impedance_controller_ = CartesianImpedanceController();
    exploration_generator_ = ExplorationGenerator();


  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeutralFlexionAxisController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  nullspace_damping_ = get_node()->get_parameter("nullspace_damping").as_double();
  max_force_ = get_node()->get_parameter("max_force").as_double();
  max_torque_ = get_node()->get_parameter("max_torque").as_double();

  // get franka robot model for jacobian and pose and 

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                     arm_id_ + "/robot_state"));
  
  // get force torque sensor readings for admittance system

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


//needed states: velocities and position
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


Eigen::Matrix<double, 6, 1> N
  nullspace_damping_ = get_node()->get_parameter("nullspace_damping").as_double(NeutralFlexionAxisController::saturateWrench(
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

Eigen::Matrix<double, kNumJoints, 1> NeutralFlexionAxisController::readRobotState(){
  for (int i = 0; i < kNumJoints; ++i) {
    q(i) = state_interfaces_[2 * i].get_value();
    dq(i) = state_interfaces_[2 * i + 1].get_value();
  }
  return q, dq;

}

void NeutralFlexionAxisController::updateCalibration(){
  
}



// externe Kraft/Moment im K-("EE"-)Frame, wie von franka_hardware geschätzt
Eigen::Vector3d NeutralFlexionAxisController::readFTSensor(){
  Eigen::Vector3d f_ext_k(force_torque_sensor_->get_forces()[0],
                          force_torque_sensor_->get_forces()[1],
                          force_torque_sensor_->get_forces()[2]);
  Eigen::Vector3d tau_ext_k(force_torque_sensor_->get_torques()[0],
                             force_torque_sensor_->get_torques()[1],
                             force_torque_sensor_->get_torques()[2]);
  return f_ext_k, tau_ext_k;

}

enum class ExperimentState
{
    CALIBRATION,
    READY,
    EXPLORATION,
    FINISHED,
    ERROR
};


// MOST IMPORTANT FUNTION (ACTUAL CONTROLLER):
// outer loop (lower freq): excitation -> F_ext -> estimate F_interaction from F_interaction = f_ext_k - F_grav_leg - F_coriolis
// F_interaction -> virtual admittance system -> x_d, dx_d 
// inner loop (higher freq): x_d, dx_d -> cartesian impedance controller
// more compliant/safe than just commmon PD control)
void NeutralFlexionAxisController::updateExploration(Eigen::Vector3d f_ext_k, Eigen::Vector3d F_g_leg, Eigen::Vector3d F_coriolis){

  // --- Outer loop: Generiere x_d und dx_d über admittanz und generiere anregung in verschiedenste richtungen
  if admittance_timer_{
    exploration_generator_.excite(); // hier noch logik ausbessern; excitation -> sense forces -> feed into admittance (so need to wait one update cycle)
    F_interaction = f_ext_k - F_g_leg - F_coriolis;
    x_d, dx_d = admittance_.update(F_interaction, dt);
  }
  
  // --- inner loop: Cartesian Impedance Controller für compliant trajectory "tracking"
  F_cmd = impedance_controller_.compute_wrench(x_d, dx_d, x, dx);


  // --- Nullspace Damping (7 Gelenke, 6 kartesische Zwänge -> 1 Redundanz) ---
  Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * (J * J.transpose()).inverse();
  Eigen::Matrix<double, 7, 7> N =
  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;
  Eigen::Matrix<double, 7, 1> tau_null = N.transpose() * (-nullspace_damping_ * dq);

  // --- Drehmomentkommando (kompensiere gravitationskraft des beins) --
  Eigen::Matrix<double, 7, 1> tau_cmd = J.transpose() * F_cmd + coriolis + tau_null + tau_grav_femur;

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }
}


controller_interface::return_type NeutralFlexionAxisController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  // --- Get State and Force/Torque readings ------------------------------------------------
  Eigen::Matrix<double, kNumJoints, 1> q, dq = readRobotState();
  Eigen::Vector3d f_ext_k, tau_ext_k = readFTSensor();
  
  
  // get jacobian w.r.t end effector frame (to base frame)   
  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

  // get coriolis force vector (for interaction force compensation)

  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // Get cartesian end effector pose from homogeneous transformation matrix
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix4d> T_ee_in_base(pose_array.data());
  Eigen::Vector3d x = ;

  // --- Calculate cartesian velocity from joint velocities -----------------------------------
  Eigen::Matrix<double, 6, 1> dx = J * dq;  // in Basis-Frame


  //State machine

  switch (state_){

    case CALIBRATION:
      updateCalibration(); // get F_g applied by leg (has to be subtracted from total external force
                          // to isolate the interaction forces; probably model free; otherwise estimate mass and schwerpunkt)
    break;

    case READY:
      updateReady();
    break;

    case EXPLORATION: // outer loop (lower freq): excitation -> F_ext -> get F_interaction
                      // F_interaction -> virtual admittance system -> x_d, dx_d 
                      // inner loop (higher freq): x_d, dx_d -> cartesian impedance controller
                      // more compliant/safe than just commmon PD control)
      updateExploration();
    break;

    case FINISHED:
      holdSafePosition(); // compensate leg gravity to hold position (robot gravity already compensated by robot)
    break;

    case ERROR:
      applySafeCommand(); // probably just move to safe position / gravity compensation
    break;
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
