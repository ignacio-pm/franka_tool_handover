#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <dmp/Trajectory.hpp>
#include <sensor_msgs/JointState.h>
#include <robot_module_msgs/JointCommand.h>

namespace franka_tool_handover {

class JointAction {
  protected:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<franka_tool_handover::JointImpedanceAction> as_;
    std::string action_name_;
    franka_tool_handover::JointImpedanceActionFeedback feedback_;
    franka_tool_handover::JointImpedanceActionResult result_;
    sensor_msgs::JointState joint_states_;
    
    ros::Subscriber sub = node_handle_.subscribe("/franka_state_controller/joint_states", 100, 
        &JointAction::statesCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    ros::Publisher command_pub = node_handle_.advertise<robot_module_msgs::JointCommand>("/joint_command", 100);
  
  public:
    JointAction(std::string name);
    ~JointAction();

    void executeCB(const franka_tool_handover::JointImpedanceGoalConstPtr &goal);

    void statesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    static int executeClient(DmpBbo::Trajectory trajectory);
};

} // namespace franka_tool_handover