#include <iostream>
#include <array>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "franka_tool_handover/qb_hand.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"

namespace franka_tool_handover {

  QbHand::QbHand(std::string name)
  {
    hand_name = name;
  }

  QbHand::~QbHand(void){}

  void QbHand::publisher(const std::string &action) {
    trajectory_msgs::JointTrajectoryPoint point;
    trajectory_msgs::JointTrajectory trajectory;
    std::vector<std::string> names;
    std::string publisher_node = "/" + hand_name + "/control/" + hand_name + "_synergy_trajectory_controller/command";
    std::string goal_node = "/" + hand_name + "/control/" + hand_name + "_synergy_trajectory_controller/follow_joint_trajectory/goal";
    ros::Publisher traj_pub = node_handle_.advertise<trajectory_msgs::JointTrajectory>(publisher_node, 1000, true);
    ros::Publisher goal_pub = node_handle_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(goal_node, 1000, true);
    ros::Rate loop_rate(100);

    names.push_back(hand_name + "_synergy_joint");
    double opening_time = 2.5;
    double closing_time = 5.0;
    std::array<double,5> positions {0.0, 0.0, 1.0, 1.0, 1.0};
    std::array<double,5> time {0.0, closing_time-1.0, closing_time, closing_time + 1.0, closing_time + 2.0};
    if (action == "open") {
      positions = {1.0, 1.0, 0.0, 0.0, 0.0};
      time = {0.0, opening_time-1.0, opening_time, opening_time + 1.0, opening_time + 2.0};
    }

    // for (int j=0; j<5; j++) {
    //   point.positions.push_back(positions[j]);
    //   // point.velocities.push_back(0.2);
    //   point.time_from_start = ros::Duration(time[j]); 
    //   trajectory.points.push_back(point);
    //   point = trajectory_msgs::JointTrajectoryPoint();
    // }

    // If only one position is sent
    float time_start = 0.5;
    point.positions.push_back(positions[4]);
    // point.velocities.push_back(0.3);
    point.time_from_start = ros::Duration(time_start); 
    trajectory.points.push_back(point);

    trajectory.joint_names = names;
    trajectory.header.stamp = ros::Time::now();

    while(traj_pub.getNumSubscribers() < 1) {
      ROS_INFO("Waiting");
      loop_rate.sleep();
    }
    traj_pub.publish(trajectory);


    // if(traj_pub) {
    //   traj_pub.publish(trajectory);
    // }
    // else {
    //   ROS_WARN("Publisher not ready!");
    // }

    // control_msgs::FollowJointTrajectoryActionGoal control_goal;

    // control_goal.goal.trajectory = trajectory;
    // control_goal.header.stamp = ros::Time(0);

    // while(goal_pub.getNumSubscribers() < 1) {
    //   ROS_INFO("Waiting 2");
    //   loop_rate.sleep();
    // }


    // if(goal_pub) {
    //   goal_pub.publish(control_goal);
    // }
    // else {
    //   ROS_WARN("Publisher not ready!");
    // }

    
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
  ros::init(argc, argv, "trajectory_publisher");

  franka_tool_handover::QbHand hand_object("qbhand1");
  bool hasPublished = false;
  ros::Rate loop_rate(100);
  //ros::Time time_program_end = ros::Time::now() + ros::Duration(0.1);
  while (ros::ok()) {
      if(!hasPublished) { 
        hand_object.publisher(action); 
        ROS_INFO("Sent");
      }
      ros::spinOnce();
      loop_rate.sleep();
      hasPublished = true;
      ROS_INFO("spin");
      
  }
  return 0;
}
