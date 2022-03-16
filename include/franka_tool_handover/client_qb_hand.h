#ifndef PUBLISHER_QB_HAND_H
#define PUBLISHER_QB_HAND_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace franka_tool_handover {

class QbHand {
  protected:
    ros::NodeHandle node_handle_;
    std::string hand_name;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac;
    std::vector<std::string> names;
    std::string action_node;
  
  public:
    QbHand(std::string name);
    ~QbHand();

    void client(const std::string &action, const double &time_action);

};

} // namespace franka_tool_handover

#endif