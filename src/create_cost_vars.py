#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from franka_tool_handover.msg import CostVariables

class Cost_file(object):

    def __init__(self,file_name):
        self.file = file_name
        self.n_joints = 7
        self.time_frame = np.array([0.0])
        open(self.file, 'w').close()
        rospy.Subscriber('/cost_vars', CostVariables, self.cost_callback, tcp_nodelay=True)


    def cost_callback(self, cost_data):
        # Frequency of the publisher is 1 kHz
        self.time_frame += 0.001
        wrenches = np.array(cost_data.wrenches)
        effort = np.array(cost_data.effort)


        cost_vars = np.concatenate((self.time_frame, np.concatenate((wrenches, effort), axis=None)), axis=None).reshape((1,-1))
        print(cost_vars.shape)

        with open(self.file, 'a') as f:
            np.savetxt(f, cost_vars, fmt='%1.3f')

if __name__ == '__main__':
    try:
        rospy.init_node('create_cost_vars')
        cost_class = Cost_file('test')

        rospy.loginfo("Subscriber \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 