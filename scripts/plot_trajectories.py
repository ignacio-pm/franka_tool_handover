from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

fig,(ax1,ax2) = plt.subplots(2)

# file = "../dmpbbo/demo_robot/results/trajectory.txt"
# fig.suptitle('Optimized trajectory')


# file = '../dmpbbo/demo_robot/trajectory.txt'
# fig.suptitle('Train trajectory')


# file = "../dmpbbo/demo_robot/results/tune_exploration_6.0/rollout004/trajectory.txt"
# fig.suptitle('Executed Trajectory')

# ts = np.loadtxt(file, usecols = 0)
# positions = np.loadtxt(file, usecols = np.arange(1,8))
# velocities = np.loadtxt(file, usecols = np.arange(8,15))
# accelerations = np.loadtxt(file, usecols = np.arange(15,22))
# impedance = np.loadtxt(file, usecols = np.arange(22,29))

# fig.subplots_adjust(hspace=.4)
# ax1.plot(ts,positions)
# ax1.set_title('Positions')
# # ax1.legend()
# ax2.plot(ts,velocities)
# ax2.set_title('Velocities')
# plt.show()

# fig.subplots_adjust(hspace=.4)
# ax1.plot(ts,accelerations)
# ax1.set_title('Accelerations')
# # ax1.legend()
# ax2.plot(ts,impedance)
# ax2.set_title('Impedance')
# # ax2.legend()
# plt.show()


file = "../dmpbbo/demo_robot/results/tune_exploration_6.0/rollout004/cost_vars.txt"
fig.suptitle('Cost Variables')

ts = np.loadtxt(file, usecols = 0)
wrenches = np.loadtxt(file, usecols = np.arange(1,7))
efforts = np.loadtxt(file, usecols = np.arange(7,14))

fig.subplots_adjust(hspace=.4)
ax1.plot(ts,wrenches)
ax1.set_title('Wrenches')
ax1.set_ylabel('N or Nm')
ax1.legend(['Force x', 'Force y', 'Force z', 'Torque x', 'Torque y', 'Torque z'])
ax2.plot(ts,efforts)
ax2.set_title('Efforts')
ax2.set_ylabel('Nm')
ax2.set_xlabel('s')
# ax2.legend()
plt.show()
