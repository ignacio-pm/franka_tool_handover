#include <iostream>
#include <array>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

void help()
{
  std::cout << "Usage: file + a string: open for opening the hand and close for closing" << std::endl;
}

int main(int argc, char **argv)
{
  if (argc != 2 || (std::string(argv[1]) != std::string("close") && std::string(argv[1]) != std::string("open")))
  {
    help();
    return -1;
  }
  std::string action = std::string(argv[1]);
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle n;
  ros::Publisher traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1000);

  ros::Rate loop_rate(100);

  trajectory_msgs::JointTrajectoryPoint point;
  trajectory_msgs::JointTrajectory trajectory;
  std::vector<std::string> names;

  names.push_back("qbhand1_synergy_joint");
  // std::cout << "Enter 0 for opening the hand and 1 for closing: " << std::endl;
  // int action;
  // std::cin >> action;
  double opening_time = 1.8;
  double closing_time = 5.0;
  std::array<double,5> positions {0.0, 0.0, 1.0, 1.0, 1.0};
  std::array<double,5> time {0.0, closing_time-0.5, closing_time, closing_time + 1.0, closing_time + 2.0};
  if (action == "open") {
    positions = {1.0, 1.0, 0.0, 0.0, 0.0};
    time = {0.0, opening_time-0.5, opening_time, opening_time + 1.0, opening_time + 2.0};
  }

  for (int j=0; j<5; j++) {
    point.positions.push_back(positions[j]);
    // point.velocities.push_back(0.2);
    point.time_from_start = ros::Duration(time[j]); 
    trajectory.points.push_back(point);
    point = trajectory_msgs::JointTrajectoryPoint();
  }

  // If only one position is sent
  // float time_start = 0.5;
  // int index;
  // std::cout << "Choose between opening hand (0) or closing (1):";
  // std::cin >> index;
  // point.positions.push_back(positions[index]);
  // // point.velocities.push_back(0.3);
  // point.time_from_start = ros::Duration(time_start); 
  // trajectory.points.push_back(point);

  trajectory.joint_names = names;
  trajectory.header.stamp = ros::Time::now();

  ros::Time time_program_end = ros::Time::now() + ros::Duration(time.back() + 0.1);

  while (ros::ok() and ros::Time::now() < time_program_end) {
      traj_pub.publish(trajectory);
      ros::spinOnce();
      loop_rate.sleep();
  }
  
  return 0;
}