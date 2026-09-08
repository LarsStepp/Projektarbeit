#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace lars_robot_controller {

/**
 * Implementation of cartesian Impedance controller
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  double elapsed_time_{0.0};
  void updateJointStates();
};

}