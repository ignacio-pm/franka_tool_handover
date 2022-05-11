#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState

class Trajectory_file(object):

    def __init__(self):
        self.n_joints = 7
        self.time_frame = 0.0
        self.prev_vel = np.zeros(self.n_joints)
        self.first_time = 0.0
        # The stiffness is set to 50 because it works better with the exploration factor 10
        self.stiffness = np.array([50, 50, 50, 50, 50, 50, 50])
        # '../dmpbbo/demo_robot/trajectory.txt'
        # 'trajectory.txt'
        open('../dmpbbo/demo_robot/results/experiment_giver/trajectory.txt', 'w').close()
        # rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.trajectory_callback, tcp_nodelay=True)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_callback, tcp_nodelay=True)

    def state_callback (self, state):
        joint_positions = np.array(state.q)
        joint_velocities = np.array(state.dq)

        # 5 is paused 3 is moving
        if state.robot_mode == 3:
            # First time step: Velocity and acceleration = 0
            if self.first_time == 0.0: 
                joint_velocities  = np.zeros(self.n_joints)
                self.first_time = state.header.stamp.to_sec()
            self.time_frame = state.header.stamp.to_sec() - self.first_time
            # Frequency is 100 Hz
            joint_accelerations = (joint_velocities - self.prev_vel) / 0.01
            self.prev_vel = np.array(state.dq)
            rospy.loginfo(self.time_frame)

            trajectory = np.concatenate((self.time_frame, np.concatenate((joint_positions, np.concatenate((joint_velocities, 
                np.concatenate((joint_accelerations, self.stiffness), axis=None)), axis=None)), axis=None)), axis=None).reshape((1,-1))

            with open('../dmpbbo/demo_robot/results/experiment_giver/trajectory.txt', 'a') as f:
                np.savetxt(f, trajectory, fmt='%1.6f')

if __name__ == '__main__':
    try:
        rospy.init_node('create_trajectory')
        traj_class = Trajectory_file()

        rospy.loginfo("Subscriber \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 