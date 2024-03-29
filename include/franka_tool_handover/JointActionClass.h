#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <dmp/Trajectory.hpp>
#include <sensor_msgs/JointState.h>
#include <robot_module_msgs/JointCommand.h>
#include <franka_msgs/FrankaState.h>
#include <franka_tool_handover/CostVariables.h>

namespace franka_tool_handover {

class JointAction {
  protected:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<franka_tool_handover::JointImpedanceAction> as_;
    std::string action_name_;
    franka_tool_handover::JointImpedanceActionFeedback feedback_;
    franka_tool_handover::JointImpedanceActionResult result_;
    franka_tool_handover::CostVariables cost_vars_;
    bool error_detected_;
    
    ros::Subscriber sub = node_handle_.subscribe("/franka_state_controller/franka_states", 100, 
        &JointAction::statesCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    ros::Publisher command_pub = node_handle_.advertise<robot_module_msgs::JointCommand>("/joint_command", 100);
  
  public:
    JointAction(std::string name);
    ~JointAction();

    void executeCB(const franka_tool_handover::JointImpedanceGoalConstPtr &goal);

    void statesCallback(const franka_msgs::FrankaStateConstPtr &msg);
    static int executeClient(DmpBbo::Trajectory trajectory);
};

} // namespace franka_tool_handover