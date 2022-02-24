from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

fig,(ax1,ax2) = plt.subplots(2)

# file = "../dmpbbo/demo_robot/results/trajectory.txt"
# fig.suptitle('Optimized trajectory')


# file = '../dmpbbo/demo_robot/trajectory.txt'
# fig.suptitle('Train trajectory')


# file = "../dmpbbo/demo_robot/results/cost_vars.txt"
# fig.suptitle('Executed trajectory')

# ts = np.loadtxt(file, usecols = 0)
# positions = np.loadtxt(file, usecols = np.arange(1,8))
# velocities = np.loadtxt(file, usecols = np.arange(8,15))
# # accelerations = np.loadtxt(file, usecols = np.arange(15,22))

# fig.subplots_adjust(hspace=.4)
# ax1.plot(ts,positions)
# ax1.set_title('Positions')
# # ax1.legend()
# ax2.plot(ts,velocities)
# ax2.set_title('Velocities')
# # ax2.legend()
# plt.show()


file = "../dmpbbo/demo_robot/results/cost_vars.txt"
fig.suptitle('Cost Variables')

ts = np.loadtxt(file, usecols = 0)
wrenches = np.loadtxt(file, usecols = np.arange(1,7))
efforts = np.loadtxt(file, usecols = np.arange(7,14))

fig.subplots_adjust(hspace=.4)
ax1.plot(ts,wrenches)
ax1.set_title('Wrenches')
# ax1.legend()
ax2.plot(ts,efforts)
ax2.set_title('Efforts')
# ax2.legend()
plt.show()
