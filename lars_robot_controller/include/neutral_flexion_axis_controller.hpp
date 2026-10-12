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
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <franka_semantic_components/franka_robot_model.hpp>
#include <semantic_components/force_torque_sensor.hpp>

#include "admittance_system.hpp"
#include "exploration_generator.hpp"
#include "flexion_guidance.hpp"
#include "impedance_controller.hpp"
#include "types.hpp"

namespace neutral_axis_controllers {

enum class ExperimentState { CALIBRATION, READY, EXPLORATION, FINISHED, ERROR };

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
  static constexpr double kGravity = 9.81;

  // Alle Größen eines Zyklus, werden einmal in update() berechnet und an die States gereicht
  struct ControlInput {
    Eigen::Matrix<double, kNumJoints, 1> q, dq, coriolis;
    Eigen::Matrix<double, 6, kNumJoints> J;
    Pose x;                 // EE-Pose im Basis-Frame
    Vector6d dx;            // EE-Twist im Basis-Frame (J*dq)
    Eigen::Matrix3d R;      // Rotation EE -> Basis
    Wrench F_ext_base;      // geschätzte externe Kraft (Franka) im Basis-Frame
    Wrench F_g_leg;         // Kraft des Beins auf den Roboter (Gravitation), Basis-Frame
    double dt = 0.0;
    double t_now = 0.0;     // ROS-Zeit in s (für Timeout des Kniewinkels)
  };

  // ---- Zustandsmaschine ------------------------------------------------
  Wrench updateCalibration(const ControlInput& in, const Eigen::Vector3d& f_ext_k,
                           const Eigen::Vector3d& tau_ext_k);
  Wrench updateReady(const ControlInput& in);
  Wrench updateExploration(const ControlInput& in);
  Wrench holdSafePosition(const ControlInput& in);
  Wrench applySafeCommand(const ControlInput& in);

  void enterState(ExperimentState s, const Pose& x);
  bool finishCalibration();
  Wrench legGravityWrench(const Eigen::Matrix3d& R) const;

  // ---- Hilfsfunktionen --------------------------------------------------
  void readRobotState(Eigen::Matrix<double, kNumJoints, 1>& q,
                      Eigen::Matrix<double, kNumJoints, 1>& dq) const;
  void readFTSensor(Eigen::Vector3d& f_ext_k, Eigen::Vector3d& tau_ext_k) const;
  Wrench saturateWrench(const Wrench& F) const;
  Eigen::Matrix<double, kNumJoints, 1> computeTorque(const ControlInput& in,
                                                      const Wrench& F_cmd) const;

  // ---- Franka Semantic Components -------------------------------------
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // ---- Regelungs-Bausteine ---------------------------------------------
  AdmittanceSystem admittance_;
  CartesianImpedanceController impedance_controller_;
  ExplorationGenerator exploration_generator_;
  FlexionGuidance guidance_;

  // ---- Parameter -----------------------------------------------------------
  std::string arm_id_ = "fr3";
  double nullspace_damping_ = 2.0;
  double max_force_ = 40.0;
  double max_torque_ = 8.0;
  double calibration_duration_ = 5.0;   // s
  double exploration_duration_ = 60.0;  // s
  double max_tracking_error_ = 0.10;    // m, Abstand x_d <-> x bis ERROR
  int outer_loop_divider_ = 10;         // Admittanz läuft mit f_update / divider
  double max_guidance_force_ = 10.0;    // N, Kraft entlang der Tangente -> ERROR
  double max_interaction_force_ = 30.0; // N, |F_interaction| (trans.) -> ERROR
  double max_sweep_angle_ = 2.0;        // rad, überstrichener Winkel -> FINISHED
  bool use_optitrack_progress_ = false;
  double target_knee_angle_ = 0.087;    // rad, FINISHED wenn Kniewinkel <= Zielwinkel
  double knee_angle_timeout_ = 0.5;     // s
  bool estimate_leg_ = true;            // false: Kalibrierung mittelt nur den F/T-Offset (Test ohne Bein)
  double deadband_force_ = 0.0;         // N, Totzone auf F_interaction
  double deadband_torque_ = 0.0;        // Nm

  // ---- Zustand -----------------------------------------------------------
  ExperimentState state_ = ExperimentState::CALIBRATION;
  double state_time_ = 0.0;
  int cycle_counter_ = 0;
  double outer_dt_acc_ = 0.0;
  Pose x_hold_;
  Wrench F_explore_ = Wrench::Zero();
  Wrench F_interaction_ = Wrench::Zero();
  std::atomic<bool> start_requested_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;

  // ---- Fortschrittsgröße (Kniewinkel von OptiTrack, Topic in GRAD, 0 = gestreckt) ------
  struct KneeAngleSample {
    double angle_rad = 0.0;
    double stamp = -1e9;  // s
  };
  realtime_tools::RealtimeBuffer<KneeAngleSample> knee_angle_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr knee_angle_sub_;

  GuidanceOutput last_guidance_;
  double last_sweep_ = 0.0;
  double last_knee_angle_ = 0.0;

  // ---- Beinmodell (Ergebnis der Kalibrierung) ----------------------------
  double leg_mass_ = 0.0;                          // kg
  Eigen::Vector3d leg_com_k_ = Eigen::Vector3d::Zero();  // Schwerpunkt im K-Frame
  // Akkumulatoren (Least Squares über statische Samples)
  double cal_aa_ = 0.0, cal_af_ = 0.0;
  Eigen::Matrix3d cal_AtA_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d cal_Atb_ = Eigen::Vector3d::Zero();
  int cal_samples_ = 0;
  Eigen::Vector3d cal_sum_f_ = Eigen::Vector3d::Zero();    // Summen für Offset-Mittelung (K-Frame)
  Eigen::Vector3d cal_sum_tau_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_f_ = Eigen::Vector3d::Zero();       // F/T-Offset (K-Frame), wird abgezogen
  Eigen::Vector3d bias_tau_ = Eigen::Vector3d::Zero();

  // ---- Logging (Realtime-Publisher, siehe INTEGRATION.md) -------------
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      rt_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      rt_wrench_pub_;
  // Debug/Auswertung: [state, sweep, knee_angle, F_int(6), x_d.p(3), x.p(3), v_ref(6), F_g_leg(6)]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr guidance_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      rt_guidance_pub_;
};

}  // namespace neutral_axis_controllers
