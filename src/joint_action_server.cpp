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
      result_.result.final_cost = cost_vars_;
      result_.result.action_completed = true;
      as_.setSucceeded(result_.result);
    }

  }

  void JointAction::statesCallback(const franka_msgs::FrankaStateConstPtr &msg) {
    cost_vars_.position = msg->q;
    cost_vars_.velocity = msg->dq;
    cost_vars_.wrenches = msg->O_F_ext_hat_K;
    cost_vars_.effort = msg->tau_J;
  }

} // namespace franka_tool_handover

int main(int argc, char** argv) {
  ros::init(argc, argv, "JointAS_rec");

  franka_tool_handover::JointAction joint_object("JointAS_rec");
  ros::spin();

  return 0;
}