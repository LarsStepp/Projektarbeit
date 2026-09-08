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
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  /* implement and initialize action server node that receives trajectory*/

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (int i = 1; i <= num_joints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    /* Get cartesian velocity and position of TCP*/
    for (int i = 1; i <= num_joints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
  return config;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/){
        arm_id_ = get_node()->get_parameter("arm_id").as_string();
        auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
        auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

        /*Need this to get the jacobian*/
        franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
            franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));
                                                    /*k_robot_model_interface_name = "robot_model";
                                                      k_robot_state_interface_name = "robot_state"*/
        
    
}




controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/,
    const rclcpp::Duration&){
        /*Implement cartesian impedance controller*/

        /* get cartesian pose x_d from the 4x4 homogeneous transform provided in franka_robot_model_->Pose()*/

        /*get Jacobian of TCP J_TCP from franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector)*/

        /* get q_dot from state interface*/

        /* calculate cartesian velocity of TCP from x_dot = J_TCP*q_dot */


        /* get x_d and x_dot_d from trajectory action server buffer*/



        /* implement formula: tau = K_gains*(x_d-x) + D_gains*(x_dot_d-x_dot) */
        /*return calculated joint torques to the hardware interfaces (effort interfaces)*/
        return controller_interface::return_type::OK;
    }



controller_interface::CallbackReturn ModelExampleController::on_activate(
const rclcpp_lifecycle::State& /*previous_state*/) {
franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}




}