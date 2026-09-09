# include <lars_robot_controller/first_controller.hpp>

#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>


namespace lars_robot_controller {

controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::string>("robot_type", "fr3");

    auto_declare<std::vector<double>>(
        "k_gains", {1500.0, 1500.0, 1500.0, 80.0, 80.0, 80.0});
    auto_declare<std::vector<double>>(
        "d_gains", {80.0, 80.0, 80.0, 8.0, 8.0, 8.0});

    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
    robot_type_ = get_node()->get_parameter("robot_type").as_string();

    const std::string model_prefix = arm_prefix_ + robot_type_;
    franka_robot_model_ =
        std::make_unique<franka_semantic_components::FrankaRobotModel>(
            model_prefix + "/robot_model",
            model_prefix + "/robot_state");

    ft_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
        arm_prefix_ + robot_type_ + "_tcp");

    return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}


controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(
        arm_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(
        arm_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(
        arm_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // The semantic components below also need these interfaces.
  if (franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  if (ft_sensor_) {
    for (const auto& name : ft_sensor_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }

  return config;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/){
        arm_id_ = get_node()->get_parameter("arm_id").as_string();
        arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
        robot_type_ = get_node()->get_parameter("robot_type").as_string();
        auto k = get_node()->get_parameter("k_gains").as_double_array();
        auto d = get_node()->get_parameter("d_gains").as_double_array();

        /*Need this to get the jacobian*/
        franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
            franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));
                                                    /*k_robot_model_interface_name = "robot_model";
                                                      k_robot_state_interface_name = "robot_state"*/
        
        if (k.size() != 6 || d.size() != 6) {
          RCLCPP_ERROR(
              get_node()->get_logger(),
              "k_gains and d_gains must contain exactly 6 values.");
          return CallbackReturn::ERROR;
        }   
    
}




controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/,
    const rclcpp::Duration&){
        /*Implement cartesian impedance controller*/

        /* get cartesian pose x from the 4x4 homogeneous transform provided in franka_robot_model_->Pose()*/
        std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
        double x_TCP = pose[12];
        double y_TCP = pose[13];
        double z_TCP = pose[14];

        /* project K and D from knee frame into world frame s.t. computation
        is carried out in world frame and compliant behavior orthogonal to tangent*/

        /*get Jacobian of TCP J_TCP from franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector)*/
        std::array<double, 42> J_TCP =
        franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);


        /* get q_dot from state interface*/
      
        

        /* calculate cartesian velocity of TCP from x_dot = J_TCP*q_dot */
      
        x_dot = J_TCP * q_dot;

        /* get x_d and x_dot_d from trajectory action server buffer*/



        /* implement formula: tau = K_gains*(x_d-x) + D_gains*(x_dot_d-x_dot) */


        /*return calculated joint torques to the hardware interfaces (effort interfaces)*/
        return controller_interface::return_type::OK;
    }



controller_interface::CallbackReturn ModelExampleController::on_activate(
const rclcpp_lifecycle::State& /*previous_state*/) {
franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
ft_sensor_->assign_loaned_state_interfaces(state_interfaces_);
return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  ft_sensor_->release_interfaces();
  for (auto& command : command_interfaces_) {
    command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}




}