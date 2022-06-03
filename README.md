This repository was started on the context of the Master's thesis "Learning control for a robot-to-robot tool handover" and contains a joint impedance controller that opens a qbSofthand when it reaches a threshold contact. The submodule repository [dmpbbo](https://github.com/ignacio-pm/dmpbbo) optimizes the trajectory with the algorithm PI<sup>BB</sup> or PI<sup>BB</sup>-CMA. 

The main branch master contains the controller for a robot in a giver task. The branch demo_receiver has the changes necessary to control a robot that receives an object. 2_robots and combined_node branches implement the use of two robots in the same computer or ROS environment. This branches do not have a working version. 

# How to: Install
The first step to install this package is to set up ROS and install the *franka_ros* package from their [repo](https://frankaemika.github.io/docs/). We used ROS Melodic and Noetic. Compatibility with ROS 2 is not granted.

After installing *franka_ros* package, there is one setting that should be change in the package. In the files *default_combined_controller.yaml* and *default_controller.yaml* and *franka_state_controller.cpp*, the publish rate should be changed from 30 to 100. It should be checked that ROS can handle this rate or similar in the computer.

After this, the following packages have to be cloned in the catkin workspace: [qbsofthand](http://wiki.ros.org/qb_hand/Tutorials/ROS%20Packages%20Installation), [robot_module_msgs](https://github.com/ReconCycle/robot_module_msgs) and this repository. After installing these repositories, initialize and update the submodule dmpbbo of this repository and you are ready to compile the packages with *catkin_make*.


# How to: Use

The Franka Emika library *franka\_ros*, the
*dmpbbo* library, to which the *dmpbbo* repository is
forked, and the library developed by QbRobotics *qb\_hand* are used as the base libraries of the programming project.

Before executing the optimization, the trajectory has to be
demonstrated. The demonstration can be done by the manual guiding of the
Franka Emika robot. The python script *create\_trajectory\_file.py*
saves the trajectory and constant impedance gains to an input file given
as an argument. After demonstrating the trajectory, the function
approximators can be trained using the file
*step1A\_trainDmpFromTrajectoryFile.cpp*. The task details are in the
python file *TaskHandoverTool.py*, and it is saved to a file object with
the script *step2\_defineTask.py*. The files called in the third step
allow the tuning of the initial exploration factor. The results of these
steps are used in the optimization.

On the highest level of the optimization execution, a bash script called
*demo\_robot.bash* iterates through the updates of the algorithm. This
script executes the initial while loop of the algorithm PI^BB^-CMA,
shown in algorithm \[alg:PIBB\]. In our case, the while loop is changed
for a for loop because the number of updates is limited. The process is
divided into two files: the execution of the roll-outs in
*./step4B\_performRollouts.bash* and the exploration, cost evaluation,
and update of the policy parameters in
*step4A\_oneOptimizationUpdate.py*. At the end of the optimization, a
file is used to plot the results *step4C\_plotOptimization.py*.

Inside the update file, the user selects the distribution updating and
its parameters. The function runs an update step of the optimization
according to the distribution method and the update number. First, the
costs and policies of the previous roll-outs are calculated. Then, the
mean and the covariance matrix of the policy distribution are updated.
Finally, the algorithm performs the exploration of the following update
with the new policy distribution and saves the policies in its folder.

The roll-outs file execution consists of Robot Operating System (ROS)
commands. The reading of section \[sec:ROS\] is recommended to
understand the ROS nomenclature.

This figure shows the nodes running on the ROS environment
of a robot. In the robot-to-robot handover, each robot has his own
environment to avoid collision between the *franka\_ros* nodes.

![image](images/rosgraph.png?raw=true "Title")

First, the ROS action server of the hand *qbhand1* is called to ensure
the correct position of the hand at the beginning of the task. Then, the
roll-out is executed with the policy, which consists of the impedance
gains and trajectory, stored in the file *dmp.xml*. The execution of the
roll-out is accomplished with a ROS action server called *JointAS* that
sends the desired policy to the controller at a rate of 1000 Hz. The ROS
node *create\_cost\_vars* runs simultaneously to save the states of the
robots at a rate of 100 Hz during the roll-out. The states of the robots
are published in the ROS topic *franka\_state\_controller* and are used
to calculate the cost of each roll-out. These states are plotted after
the roll-out execution with the file *plot\_trajectory.py*.

After finishing the roll-out, the same ROS action server as the roll-out
sends the command to the robots to go back to the starting positions. At
last, the algorithm sends an action to the hands of the robots to go
back to the starting position: open for the receiver robot and close for
the giver.

The controller is responsible for executing the trajectory sent by the
DMPs with the optimized impedance gains and sending a command via the
publisher *handover\_bool* to the hand when the handover is detected.
The controller runs on the ROS node *franka\_control*. The desired joint
positions, joint velocities, and impedance gains are received via a ROS
subscriber from the node *joint\_command* with a rate of 1000 Hz. This
data is processed in the update loop of the robot, and the impedance
controller transfers it to joint torque commands with formula
\[eq:torque\_impedance\]. This torque command is sent directly to the
master controller of the robot.

The hand action is sent in the update loop too. When the force in the
Z-axis reaches the threshold compared to the initial force sensed by the
robot, the controller sends the command to the hand action server. A ROS
publisher publishes a bool variable that the node *create\_cost\_vars*
uses to save the handover time.

