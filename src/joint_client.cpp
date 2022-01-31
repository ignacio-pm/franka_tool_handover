#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_tool_handover/JointImpedanceAction.h>
#include <dmp/Trajectory.hpp>
#include <franka_tool_handover/JointActionClass.h>
#include <vector>

using namespace Eigen;

namespace franka_tool_handover {

  int JointAction::executeClient (DmpBbo::Trajectory trajectory) {
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<franka_tool_handover::JointImpedanceAction> ac("joint_trajectory", true);
    ROS_DEBUG("Waiting for joint action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_DEBUG("Joint action server started, sending goal.");
    // send a goal to the action
    franka_tool_handover::JointImpedanceGoal goal;
    int n_time_steps = trajectory.length();
    Eigen::MatrixXd mat_pos = trajectory.ys();
    Eigen::MatrixXd mat_vel = trajectory.yds();
    Eigen::MatrixXd mat_impedance = trajectory.misc();

    assert(mat_pos.rows() == mat_vel.rows());
    for (int i = 0; i < n_time_steps; i++) {
      std::vector<double> vec_pos(mat_pos.cols());
      std::vector<double> vec_vel(mat_vel.cols());
      std::vector<double> vec_impedance(mat_vel.cols());
      Eigen::Map<Eigen::VectorXd>(vec_pos.data(), mat_pos.cols()) = mat_pos.row(i);   
      Eigen::Map<Eigen::VectorXd>(vec_vel.data(), mat_vel.cols()) = mat_vel.row(i);
      Eigen::Map<Eigen::VectorXd>(vec_impedance.data(), mat_impedance.cols()) = mat_impedance.row(i);     
      goal.joints[i].pos = vec_pos;
      goal.joints[i].vel = vec_vel;

      goal.joints[i].impedance.n = 7;
      goal.joints[i].impedance.k = vec_impedance;
      goal.joints[i].impedance.d = vec_impedance;
      // The damping gain will be calculated from the proportional k 
    }
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(45.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_DEBUG("Rollout finished: %s",state.toString().c_str());
    }
    else
      ROS_DEBUG("Rollout did not finish before the time out.");

    //exit
    return 0;
  }
} // namespace franka_tool_handover