#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <franka_tool_handover/JointActionClass.h>

using franka_hw::ServiceContainer;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_action_node");

  ros::NodeHandle node_handle;

  franka_tool_handover::JointAction actionObject();
}
