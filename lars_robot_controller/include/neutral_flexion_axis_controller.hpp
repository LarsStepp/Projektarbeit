// neutral_flexion_axis_controller.hpp
//
// Hybrid Geschwindigkeits-/Admittanzregler für franka_ros2.
// Eine EE-Achse wird mit konstanter kartesischer Geschwindigkeit geführt
// (quasi-kinematisch, über Geschwindigkeitsfehler-Rückführung), die
// restlichen 5 kartesischen Freiheitsgrade sind rein compliant (K -> 0,
// nur virtuelle Dämpfung), sodass das Unterbein die natürliche Bewegung
// um die Knieachse ausführen kann.
//
// WICHTIG: Die exakten Methodennamen von franka_semantic_components::
// FrankaRobotModel können sich zwischen franka_ros2-Versionen leicht
// unterscheiden. Prüfe sie einmal in deinem Workspace mit:
//
//   grep -n "getZeroJacobian\|getMassMatrix\|getCoriolisForceVector\|getPoseMatrix" \
//     $(ros2 pkg prefix franka_semantic_components)/include/franka_semantic_components/franka_robot_model.hpp
//
// und passe die Aufrufe unten ggf. an (Signaturen sind sonst 1:1 austauschbar).

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

  // ---- Parameter -----------------------------------------------------
  std::string arm_id_;
  int driven_axis_{5};        // 0..2 = Translation x,y,z (EE-Frame), 3..5 = Rotation x,y,z (EE-Frame)
  double axis_velocity_{0.05};        // m/s (Translation) bzw. rad/s (Rotation)
  double driven_axis_damping_{200.0}; // "steif" genug, um die Sollgeschwindigkeit zu tracken
  double free_trans_damping_{15.0};   // N·s/m auf den freien Translationsachsen
  double free_rot_damping_{1.0};      // Nm·s/rad auf den freien Rotationsachsen
  double nullspace_damping_{2.0};
  double max_force_{40.0};            // N, Sicherheitsclamp
  double max_torque_{8.0};            // Nm, Sicherheitsclamp

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
  Eigen::Matrix<double, 6, 1> buildAxisDirectionInBase(const Eigen::Matrix3d& R_ee_in_base) const;
  Eigen::Matrix<double, 6, 1> saturateWrench(const Eigen::Matrix<double, 6, 1>& F) const;
};

}  // namespace neutral_axis_controllers