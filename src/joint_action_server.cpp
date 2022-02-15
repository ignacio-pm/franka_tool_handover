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
    ros::Rate r(100);
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
      feedback_.feedback.joints = joint_states_;
      as_.publishFeedback(feedback_.feedback);
      r.sleep();
    }

    if(success) {
      result_.result.final_joints = joint_states_;
      result_.result.fully_succeeded = true;
      as_.setSucceeded(result_.result);
    }

  }

  void JointAction::statesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_states_.position = msg->position;
    joint_states_.velocity = msg->velocity;
  }

} // namespace franka_tool_handover

int main(int argc, char** argv) {
  ros::init(argc, argv, "JointAS");

  franka_tool_handover::JointAction joint_object("JointAS");
  ros::spin();

  return 0;
}