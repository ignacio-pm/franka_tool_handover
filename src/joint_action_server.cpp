#include <franka_tool_handover/joint_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <franka_tool_handover/JointActionClass.h>


namespace franka_tool_handover {

    JointAction::JointAction() :
      as_(node_handle_, std::string ("JointAction"), boost::bind(&JointAction::executeCB, this, _1), false),
      action_name_(std::string ("JointAction"))
    {
      as_.start();
    }

    JointAction::~JointAction(){}

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

    void JointAction::statesCallback(const sensor_msgs::JointState &msg) {
      joint_states_ = msg;
    }

} // namespace franka_tool_handover