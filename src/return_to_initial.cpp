#include "dmp/Trajectory.hpp"
#include "dmp/serialization.hpp"
#include "franka_tool_handover/JointActionClass.h"

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace DmpBbo;
using namespace franka_tool_handover;

void help(char* binary_name)
{
  cout << "Usage:   " << binary_name << " <input trajectory (txt)>" << endl;
  cout << "Example: " << binary_name << " trajectory.txt "<< endl;
}

/** Main function
 * \param[in] n_args Number of arguments
 * \param[in] args Arguments themselves
 * \return Success of execution. 0 if successful.
 */
int main(int n_args, char** args)
{
  
  string input_trajectory_file;
  
  if (n_args != 2)
  {
    help(args[0]);
    return -1;
  }

  input_trajectory_file = string(args[1]);
  cout << "C++    | Executing "; 
  for (int ii=0; ii<n_args; ii++) cout << " " << args[ii]; 
  cout << endl;
  
  cout << "C++    |     Reading trajectory from file: " << input_trajectory_file << endl;

  Trajectory trajectory = Trajectory::readFromFile(input_trajectory_file);
  if (trajectory.length()==0)
  {
    cerr << "ERROR: The file " << input_trajectory_file << " could not be found. Aborting." << endl << endl;
    help(args[0]);
    return -1;
  }

  Trajectory return_traj;
  return_traj = Trajectory::generateMinJerkTrajectory(trajectory.ts().head(6000), trajectory.final_y(), trajectory.initial_y());

  MatrixXd impedance_matrix(1, 7);
  impedance_matrix << 600, 600, 600, 250, 150, 150, 50;

  return_traj.set_misc(impedance_matrix);

  franka_tool_handover::JointAction::executeClient(return_traj);

  return 0;
}
