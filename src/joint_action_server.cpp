#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <franka_tool_handover/JointActionClass.h>


namespace franka_tool_handover {

  JointAction::JointAction(std::string name) :
    as_(node_handle_, name, boost::bind(&JointAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  JointAction::~JointAction(void){}

  void JointAction::executeCB(const franka_tool_handover::JointImpedanceGoalConstPtr &goal) {
    ros::Rate r(1000);
    bool success = true;

    for(const auto &command : goal->joints) {
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      command_pub.publish(command);
      feedback_.feedback.cost_vars = cost_vars_;
      as_.publishFeedback(feedback_.feedback);
      r.sleep();
    }

    if(success) {
      if (error_detected_)
        result_.result.action_completed = false;
      else
        result_.result.action_completed = true; 
      result_.result.final_cost = cost_vars_;
      as_.setSucceeded(result_.result);
    }

  }

  void JointAction::statesCallback(const franka_msgs::FrankaStateConstPtr &msg) {
    cost_vars_.position = msg->q;
    cost_vars_.velocity = msg->dq;
    cost_vars_.wrenches = msg->O_F_ext_hat_K;
    cost_vars_.effort = msg->tau_J;
    // Robot mode 4 is Reflex error and 7 is automatic error recovery
    if (msg->robot_mode == 4 || msg->robot_mode == 7)
      error_detected_ = true;
  }

} // namespace franka_tool_handover

int main(int argc, char** argv) {
  ros::init(argc, argv, "JointAS_rec");

  franka_tool_handover::JointAction joint_object("JointAS_rec");
  ros::spin();

  return 0;
}