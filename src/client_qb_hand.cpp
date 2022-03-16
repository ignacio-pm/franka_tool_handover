#include <iostream>
#include <array>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "franka_tool_handover/client_qb_hand.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>

namespace franka_tool_handover {

  QbHand::QbHand(std::string name) : 
    ac("/" + name + "/control/" + name + "_synergy_trajectory_controller/follow_joint_trajectory", true)
  {
    hand_name = name;
    action_node = "/" + hand_name + "/control/" + hand_name + "_synergy_trajectory_controller/follow_joint_trajectory";
    ROS_INFO("Waiting for QBHand action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    names.push_back(hand_name + "_synergy_joint");
  }

  QbHand::~QbHand(void){}

  void QbHand::client(const std::string &action, const double &time_action) {
    trajectory_msgs::JointTrajectoryPoint point;
    trajectory_msgs::JointTrajectory trajectory;
    control_msgs::FollowJointTrajectoryGoal action_goal;
    // double opening_time = 2.5;
    // double closing_time = 5.0;
    // std::array<double,5> positions {0.0, 0.0, 1.0, 1.0, 1.0};
    // std::array<double,5> time {0.0, closing_time-1.0, closing_time, closing_time + 1.0, closing_time + 2.0};
    // if (action == "open") {
    //   positions = {1.0, 1.0, 0.0, 0.0, 0.0};
    //   time = {0.0, opening_time-1.0, opening_time, opening_time + 1.0, opening_time + 2.0};
    // }

    // for (int j=0; j<5; j++) {
    //   point.positions.push_back(positions[j]);
    //   // point.velocities.push_back(0.2);
    //   point.time_from_start = ros::Duration(time[j]); 
    //   trajectory.points.push_back(point);
    //   point = trajectory_msgs::JointTrajectoryPoint();
    // }

    // If only one position is sent
    double time_start = time_action;
    int final_position;
    if (action == "open") {
      final_position = 0.0;
    }
    else if (action == "close") {
      final_position = 1.0;
    }
    else {
      ROS_ERROR("Function QB Hand Client was called with an invalid action: %s. Possible actions: open or close", action.c_str());
      return;
    }
    point.positions.push_back(final_position);
    // point.velocities.push_back(0.3);
    point.time_from_start = ros::Duration(time_start); 
    trajectory.points.push_back(point);

    trajectory.joint_names = names;
    trajectory.header.stamp = ros::Time::now();

    action_goal.trajectory = trajectory;

    ROS_INFO("QBHand action server started, sending action %s.", action.c_str());

    ac.sendGoal(action_goal);
  }
} // namespace franka_tool_handover

void help()
{
  std::cout << "Usage: file + a string: open for opening the hand and close for closing" << std::endl;
}


int main(int argc, char** argv) {
  if (argc != 2 || (std::string(argv[1]) != std::string("close") && std::string(argv[1]) != std::string("open")))
  {
    help();
    return -1;
  }
  std::string action = std::string(argv[1]);
  ros::init(argc, argv, "qbhand_client");

  double time_action = 1.0;

  franka_tool_handover::QbHand hand_object("qbhand1");
  hand_object.client(action, time_action); 
  return 0;
}
