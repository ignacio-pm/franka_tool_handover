#!/usr/bin/env python

import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from franka_tool_handover.msg import JointImpedanceActionFeedback

class Cost_file(object):

    def __init__(self,file_name):
        self.file = file_name
        self.n_joints = 7
        self.time_frame = 0.0
        open(self.file, 'w').close()
        rospy.Subscriber('/JointAS/feedback', JointImpedanceActionFeedback, self.feedback_callback, tcp_nodelay=True)


    def feedback_callback(self, msg):
        # Frequency of the publisher is ~1000 Hz. Not exact to improve plotting
        self.time_frame += 0.001
        wrenches = np.array(msg.feedback.cost_vars.wrenches)
        effort = np.array(msg.feedback.cost_vars.effort)
        position = np.array(msg.feedback.cost_vars.position)
        velocity = np.array(msg.feedback.cost_vars.velocity)

        cost_vars_numpy = np.concatenate((self.time_frame, np.concatenate((wrenches, effort), axis=None)), axis=None).reshape((1,-1))
        # print("Printing time frame: ", self.time_frame)

        with open(self.file, 'a') as f:
            np.savetxt(f, cost_vars_numpy, fmt='%1.4f')

if __name__ == '__main__':
    try:
        rospy.init_node('create_cost_vars')
        file = sys.argv[1] if len(sys.argv) > 1 else '../dmpbbo/demo_robot/results/cost_vars.txt'
        cost_class = Cost_file(file)

        rospy.loginfo("Subscriber \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 