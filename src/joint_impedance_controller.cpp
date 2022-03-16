// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_tool_handover/joint_impedance_controller.h>

#include <cmath>
#include <memory>
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <franka_msgs/SetLoad.h>
#include <franka_hw/services.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <franka_tool_handover/client_qb_hand.h>

#include <franka/robot_state.h>

namespace franka_tool_handover {

bool JointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {                                       
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter vel_max, defaulting to: " << vel_max_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_init_) || k_init_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_init_) || d_init_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  command_sub = node_handle.subscribe("/joint_command", 100, &JointImpedanceController::commandCallback, 
                  this, ros::TransportHints().reliable().tcpNoDelay());
  // setLoadClient = node_handle.serviceClient<franka_msgs::SetLoad>("/franka_control/set_load", true);
  prev_time = ros::Time::now();
  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  ROS_INFO("JointImpedanceController: Finished init");       
  return true;
}

void JointImpedanceController::starting(const ros::Time& /*time*/) { 
  franka::RobotState robot_state = state_handle_->getRobotState();
  for (size_t i = 0; i < 7; i++)
    {
      q_d[i] = robot_state.q[i];
      dq_d[i] = 0;
      k_gains_[i] = k_init_[i];
      d_gains_[i] = d_init_[i];
    }

  franka_msgs::SetLoad setLoaddata;
  setLoaddata.request.mass = weight_tool_;
  setLoaddata.request.F_x_center_load = boost::array<double,3> { {0.0, 0.0, 0.0} };
  setLoaddata.request.load_inertia = boost::array<double,9> { {0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0} };
  // ROS_INFO("Service info: %d", setLoadClient.call(setLoaddata));
  ROS_INFO("JointImpedanceController: Tool weight set"); 

  // A handover can not be detected until the movement starts
  handover_detected_ = true;
  initial_force_z_ = robot_state.O_F_ext_hat_K[2];
  ROS_INFO("JointImpedanceController: Started"); 
}

void JointImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  double force_z = robot_state.O_F_ext_hat_K[2];

  if(!handover_detected_ && force_z < (- 9.81 * weight_tool_ + initial_force_z_)) {
    ROS_INFO("JointImpedanceController: Handover detected"); 
    handover_detected_ = true;
    std::string action = "open";
    hand_object.client(action, 0.1); 
  //   franka_msgs::SetLoad setLoaddata;
  //   setLoaddata.request.mass = 0.770 + weight_tool_;
  //   setLoadClient.call(setLoaddata);
  }
  std::array<double, 7> gravity = model_handle_->getGravity();

  std::array<double, 7> tau_d_calculated;

  double alpha = 0.8;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (dq_d[i] - robot_state.dq[i]);
  }
  // float d_value = d_gains_[4] * (dq_d[4] - robot_state.dq[4]);
  // ROS_INFO_COND(d_value > 0.5 or d_value < -0.5 , "Info 4: %.6f, %.6f, %.6f, %3.2f", dq_d[4], robot_state.dq[4], d_gains_[4], d_value);

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> JointImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void JointImpedanceController::commandCallback(const robot_module_msgs::JointCommandConstPtr &msg) {
  // Print new target from the subscriber node command
  // ROS_DEBUG("-------------------------------------");
  // ROS_INFO("positions: %6.2f, %6.2f, %6.2f, %6.2f, (5) %6.2f, %6.2f, %6.2f", 
  // msg->pos[0], msg->pos[1], msg->pos[2], msg->pos[3], msg->pos[4], msg->pos[5], msg->pos[6]);
  // ROS_INFO("velocities: %6.2f, %6.2f, %6.2f, %6.2f, (5) %6.2f, %6.2f, %6.2f", 
  // msg->vel[0], msg->vel[1], msg->vel[2], msg->vel[3], msg->vel[4], msg->vel[5], msg->vel[6]);
  // ROS_INFO("stiffness: %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", 
  // msg->impedance.k[0], msg->impedance.k[1], msg->impedance.k[2], msg->impedance.k[3], 
  // msg->impedance.k[4], msg->impedance.k[5], msg->impedance.k[6]);
  // ROS_INFO("damping: %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", 
  // msg->impedance.d[0], msg->impedance.d[1], msg->impedance.d[2], msg->impedance.d[3], 
  // msg->impedance.d[4], msg->impedance.d[5], msg->impedance.d[6]);
  if (ros::Time::now().toSec() - prev_time.toSec() > 2.0) {
    franka::RobotState robot_state = state_handle_->getRobotState();
    initial_force_z_ = robot_state.O_F_ext_hat_K[2];
    handover_detected_ = false;
    ROS_INFO("JointImpedanceController: Detected new rollout"); 
    ROS_INFO("Initial force: %f", initial_force_z_);
  } 


  for (size_t i = 0; i < 7; i++)
  {
    if(msg->vel[i] > 2.62) {
      ROS_ERROR("Velocity over limit.");
      command_sub.shutdown();
    }
    else if(std::abs(msg->pos[i] - joint_handles_[i].getPosition()) > 0.1) {
      ROS_ERROR("Difference in position over the limit. Stopping subscriber.");
      command_sub.shutdown();
    }
    else {
      q_d[i] = msg->pos[i];
      dq_d[i] = msg->vel[i];
      k_gains_[i] = msg->impedance.k[i];
      d_gains_[i] = msg->impedance.d[i];
    }
  }
  prev_time = ros::Time::now();
}

}  // namespace franka_tool_handover

PLUGINLIB_EXPORT_CLASS(franka_tool_handover::JointImpedanceController,
                       controller_interface::ControllerBase)
