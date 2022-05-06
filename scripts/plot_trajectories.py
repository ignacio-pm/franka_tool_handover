#!/usr/bin/env python

import sys
# from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

def save_trajectory_plot(folder, prefix):

    file = folder + '/trajectory.txt'

    ts = np.loadtxt(file, usecols = 0)
    positions = np.loadtxt(file, usecols = np.arange(1,8))
    velocities = np.loadtxt(file, usecols = np.arange(8,15))
    # accelerations = np.loadtxt(file, usecols = np.arange(15,22))

    # impedance = np.loadtxt(file, usecols = np.arange(22,29))
    # imp_vector = np.array([6.0, 6.0, 6.0, 2.5, 1.5, 1.5, 0.5])
    # impedance = impedance * imp_vector

    fig,(ax1,ax2) = plt.subplots(2)
    title = prefix + ' trajectory'
    fig.suptitle(title)

    fig.subplots_adjust(hspace=.4)
    ax1.plot(ts,positions)
    ax1.set_title('Positions')
    ax1.set_ylabel('rad')
    ax2.plot(ts,velocities)
    ax2.set_title('Velocities')
    ax2.set_ylabel('rad/s')
    ax2.set_xlabel('s')
    fig.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'], loc=7)
    plt.subplots_adjust(right=0.8)
    file_name = folder + '/' + prefix.lower() + '_trajectory.png'
    plt.savefig(file_name)
    plt.close()

    # fig, ax3 = plt.subplots()
    # fig.subplots_adjust(right=0.8) 
    # fig.subplots_adjust(hspace=.4)

    # ax3.plot(ts,impedance)
    # ax3.set_ylabel('Gain Kp')
    # ax3.set_xlabel('s')
    # ax3.set_title('Last rollout impedance gains')
    # fig.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'], loc=7)
    # file_name = folder + '/' + 'impedance_gains.png'
    # plt.savefig(file_name)
    # plt.close()

def save_cost_vars_plots(folder):
    file = folder + '/cost_vars.txt'
    fig,(ax1,ax2) = plt.subplots(2)
    fig.suptitle('Executed trajectory')

    ts = np.loadtxt(file, usecols = 0)[0:-1]
    positions = np.loadtxt(file, usecols = np.arange(14,21))[0:-1]
    velocities = np.loadtxt(file, usecols = np.arange(21,28))[0:-1]

    fig.subplots_adjust(hspace=.4)
    ax1.plot(ts,positions)
    ax1.set_title('Positions')
    ax1.set_ylabel('rad')
    ax2.plot(ts,velocities)
    ax2.set_title('Velocities')
    ax2.set_ylabel('rad/s')
    ax2.set_xlabel('s')
    fig.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'], loc=7)
    plt.subplots_adjust(right=0.8)
    file_name = folder + '/positions.png'
    plt.savefig(file_name)
    plt.close()

    fig2,(ax3,ax4) = plt.subplots(2)

    fig2.suptitle('Wrenches')

    ts = np.loadtxt(file, usecols = 0)[0:-1]
    forces = np.loadtxt(file, usecols = np.arange(1,4))[0:-1]
    torques = np.loadtxt(file, usecols = np.arange(4,7))[0:-1]
    efforts = np.loadtxt(file, usecols = np.arange(7,14))[0:-1]

    fig2.subplots_adjust(hspace=.4)
    ax3.plot(ts, forces)
    ax3.set_title('Forces')
    ax3.set_ylabel('N')
    ax3.legend(['Force x', 'Force y', 'Force z'], bbox_to_anchor=(1.04,0.5), loc="center left", borderaxespad=0)
    ax4.set_prop_cycle(color=['#d62728', '#9467bd', '#8c564b'])
    ax4.plot(ts,torques)
    ax4.set_title('Torques')
    ax4.set_ylabel('Nm')
    ax4.set_xlabel('s')
    ax4.legend(['Torque x', 'Torque y', 'Torque z'], bbox_to_anchor=(1.04,0.5), loc="center left", borderaxespad=0)
    plt.subplots_adjust(right=0.78)
    file_name = folder + '/wrenches.png'
    plt.savefig(file_name)
    plt.close()

    plt.figure()
    plt.plot(ts,efforts)
    plt.title('Efforts')
    plt.ylabel('Nm')
    plt.xlabel('s')
    plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'], bbox_to_anchor=(1.04,0.5), loc="center left", borderaxespad=0)
    plt.subplots_adjust(right=0.8)
    file_name = folder + '/efforts.png'
    plt.savefig(file_name)
    plt.close()

if __name__ == '__main__':

    if len(sys.argv) != 3:
        print("This script will save the plots of the contents of a cost_vars.txt file or a trajectory.txt file.") 
        print("Usage: plot_trajectories.py <cost_vars / optimized_trajectory / train_trajectory> <contained folder without />.")
        sys.exit(0)
    type = sys.argv[1]
    folder = sys.argv[2]
    if (type == 'optimized_trajectory'):
        save_trajectory_plot(folder, 'Trained')
    elif (type == 'train_trajectory'):
        save_trajectory_plot(folder, 'Demonstrated')    
    elif (type == 'cost_vars'):
        save_cost_vars_plots(folder)
        save_trajectory_plot(folder, 'Rollout')  
    else:
        print("Wrong first argument. It should be cost_vars or optimized_trajectory or train_trajectory")
