// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_msgs/SetLoad.h>
#include <std_msgs/Bool.h>

#include <robot_module_msgs/JointCommand.h>
#include <franka_tool_handover/CostVariables.h>
#include <franka_tool_handover/client_qb_hand.h>

namespace franka_tool_handover {

class JointImpedanceController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void commandCallback(const robot_module_msgs::JointCommandConstPtr &msg);

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double vel_max_{0.05};
  double vel_current_{0.0};
  double coriolis_factor_{1.0};
  double weight_tool_{0.444};
  double initial_force_z_;
  bool handover_detected_;
  bool first_movement_detected_;
  static constexpr double kDeltaTauMax{1.0};
  ros::Time prev_time;
  ros::Time initial_time;

  std::array<double, 7> k_gains_;
  std::array<double, 7> d_gains_;
  std::array<double, 7> dq_filtered_;
  std::array<double, 7> q_d;
  std::array<double, 7> dq_d;
  std::vector<double> k_init_;
  std::vector<double> d_init_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};

  ros::Subscriber command_sub;
  ros::ServiceClient setLoadClient;
  ros::Publisher hand_pub;
  franka_tool_handover::QbHand hand_object{"qbhand1"};
  realtime_tools::RealtimePublisher<std_msgs::Bool> handover_publisher_;
};

}  // namespace franka_tool_handover
