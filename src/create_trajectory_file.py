#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState

class Trajectory_file(object):

    def __init__(self):
        self.n_joints = 7
        self.time_frame = np.array([0.0])
        # '../../dmpbbo/demo_robot/trajectory.txt'
        open('trajectory.txt', 'w').close()
        rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.trajectory_callback, tcp_nodelay=True)


    def trajectory_callback(self, joints):
        # Frequency of the publisher is 1 kHz
        self.time_frame += 0.001
        joint_positions = np.array(joints.position)
        joint_velocities = np.array(joints.velocity)
        joint_accelerations = np.zeros(self.n_joints)

        trajectory = np.concatenate((self.time_frame, np.concatenate((joint_positions, 
        np.concatenate((joint_velocities, joint_accelerations), axis=None)), axis=None)), axis=None).reshape((1,-1))
        print(trajectory.shape)

        with open('trajectory.txt', 'a') as f:
            np.savetxt(f, trajectory, fmt='%1.6f')

if __name__ == '__main__':
    try:
        rospy.init_node('create_trajectory')
        traj_class = Trajectory_file()

        rospy.loginfo("Subscriber \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 