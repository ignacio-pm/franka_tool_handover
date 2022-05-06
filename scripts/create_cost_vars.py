#!/usr/bin/env python3

import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from franka_tool_handover.msg import JointImpedanceActionFeedback
from franka_tool_handover.msg import JointImpedanceActionResult
from std_msgs.msg import Bool

class Cost_file(object):

    def __init__(self,file_name):
        self.file = file_name
        self.n_joints = 7
        self.time_frame = 0.0
        self.handover_time = 8.0
        open(self.file, 'w').close()
        rospy.Subscriber('/JointAS_rec/feedback', JointImpedanceActionFeedback, self.feedback_callback, tcp_nodelay=True)
        rospy.Subscriber('/JointAS_rec/result', JointImpedanceActionResult, self.result_callback, tcp_nodelay=True)
        rospy.Subscriber('/joint_impedance_controller_rec/handover_bool', Bool, self.handover_callback, tcp_nodelay=True)


    def feedback_callback(self, msg):
        # Frequency of the publisher is ~1000 Hz. Not exact but set to 0.001 to improve plotting
        self.time_frame += 0.001
        wrenches = np.array(msg.feedback.cost_vars.wrenches)
        effort = np.array(msg.feedback.cost_vars.effort)
        position = np.array(msg.feedback.cost_vars.position)
        velocity = np.array(msg.feedback.cost_vars.velocity)

        cost_vars_numpy = np.concatenate((self.time_frame, np.concatenate((wrenches, np.concatenate((effort, 
        np.concatenate((position,velocity),axis = None)), axis = None)), axis=None)), axis=None).reshape((1,-1))
        self.length = cost_vars_numpy.size
        # print("Printing time frame: ", self.time_frame)
        
        # np.r_[cost_vars_numpy, np.zeros(len(cost_vars_numpy[0]))]

        with open(self.file, 'a') as f:
            np.savetxt(f, cost_vars_numpy, fmt='%1.4f')

    def result_callback(self, msg):
        if msg.result.action_completed:
            # If handover_time = 8.0 the evaluate plotout will detect that the time of the handover is > max_time
            # Therefore, it will be set as not_completed
            last_line = np.full((1,self.length), self.handover_time)
        else:
            last_line = np.full((1,self.length), -1.0)
            rospy.loginfo("Error in the movement of the action server.")
        with open(self.file, 'a') as f:
                np.savetxt(f, last_line, fmt='%1.3f')
        rospy.signal_shutdown("The node is shutdown because the action was terminated")

    def handover_callback(self, msg):
        if msg:
            self.handover_time = self.time_frame

if __name__ == '__main__':
    try:
        rospy.init_node('create_cost_vars')
        file = sys.argv[1] if len(sys.argv) > 1 else '../dmpbbo/demo_robot/results/cost_vars.txt'
        cost_class = Cost_file(file)

        rospy.loginfo("Subscriber \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 