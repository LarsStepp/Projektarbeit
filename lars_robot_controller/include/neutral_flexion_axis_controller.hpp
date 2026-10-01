
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <franka_semantic_components/franka_robot_model.hpp>
#include <semantic_components/force_torque_sensor.hpp>

namespace neutral_axis_controllers {

class NeutralFlexionAxisController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

 private:
  static constexpr int kNumJoints = 7;


  // ---- Franka Semantic Components -------------------------------------
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // ---- Logging (Realtime-Publisher, siehe INTEGRATION.md) -------------
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      rt_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      rt_wrench_pub_;

  // ---- Hilfsfunktionen --------------------------------------------------
  Eigen::Matrix<double, 6, 1> saturateWrench(const Eigen::Matrix<double, 6, 1>& F) const;
};

} 