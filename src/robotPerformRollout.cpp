/**
 * \file trainPerformRollout.cpp
 * \author Freek Stulp
 * \brief  Demonstrates how to train a Dmp with a trajectory in a txt file.
 *
 * \ingroup Demos
 * \ingroup Dmps // zzz
 *
 * This file is part of DmpBbo, a set of libraries and programs for the 
 * black-box optimization of dynamical movement primitives.
 * Copyright (C) 2014 Freek Stulp, ENSTA-ParisTech
 * 
 * DmpBbo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * DmpBbo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with DmpBbo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "dmp/Dmp.hpp"
#include "dmp/Trajectory.hpp"
#include "dmp/serialization.hpp"

#include "dynamicalsystems/DynamicalSystem.hpp"
#include "dynamicalsystems/ExponentialSystem.hpp"
#include "dynamicalsystems/SigmoidSystem.hpp"
#include "dynamicalsystems/TimeSystem.hpp"
#include "dynamicalsystems/SpringDamperSystem.hpp"

#include "functionapproximators/FunctionApproximatorRBFN.hpp"
#include "functionapproximators/MetaParametersRBFN.hpp"
#include "functionapproximators/ModelParametersRBFN.hpp"
#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"
#include "functionapproximators/ModelParametersLWR.hpp"

#include "franka_tool_handover/JointActionClass.h"

#include "dmpbbo_io/EigenFileIO.hpp"

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <python3.6m/Python.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

using namespace std;
using namespace Eigen;
using namespace DmpBbo;
using namespace franka_tool_handover;

void help(char* binary_name)
{
  cout << "Usage: " << binary_name 
  << " <dmp filename.xml> <output trajectory filename.txt>  <output cost vars filename.txt> [dmp parameters.txt] [output dmp filename.xml]" << endl;
}

/** Main function
 * \param[in] n_args Number of arguments
 * \param[in] args Arguments themselves
 * \return Success of exection. 0 if successful.
 */
int main(int n_args, char** args)
{
  
  if (n_args<4 || n_args>6)
  {
    help(args[0]);
    return -1;
  }
  
  if (string(args[1]).compare("--help")==0)
  {
    help(args[0]);
    return 0;
  }

  string dmp_filename = string(args[1]);
  string traj_filename = string(args[2]);
  string cost_vars_filename = string(args[3]);
  string sample_filename = "";
  if (n_args>4)
    sample_filename = string(args[4]);
  string dmp_output_filename = "";
  if (n_args>5)
    dmp_output_filename = string(args[5]);

  cout << "C++    | Executing "; 
  for (int ii=0; ii<n_args; ii++) cout << " " << args[ii]; 
  cout << endl;
  
  // Read dmp from xml file
  cout << "C++    |     Reading dmp from file '" << dmp_filename << "'"  << endl;
  std::ifstream ifs(dmp_filename);
  boost::archive::xml_iarchive ia(ifs);
  Dmp* dmp;
  ia >> BOOST_SERIALIZATION_NVP(dmp);
  ifs.close();
  cout << "C++    |         " << *dmp << endl;

  // Read sample file, if necessary
  if (!sample_filename.empty())
  {
    cout << "C++    |     Reading sample from file '" << sample_filename << "'"  << endl;
    VectorXd sample;
    if (!loadMatrix(sample_filename, sample)) 
    {
      cerr << "C++    | WARNING: Could not read sample file. Executing default DMP instead." << endl;
    }
    else
    {
      // Set DMP parameters to sample
      dmp->setParameterVectorSelected(sample);
      // Save dmp whose parameters have been perturbed, if necessary
      if (!dmp_output_filename.empty())
      {
        cout << "C++    |     Saving dmp to file '" << dmp_output_filename << "'"  << endl;
        std::ofstream ofs(dmp_output_filename);
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp("dmp",dmp);
        ofs.close();
      }
    }
  }

  // Integrate DMP longer than the tau with which it was trained
  double integration_time = 1.5*dmp->tau();
  double frequency_Hz = 1000.0;
  cout << "C++    |     Integrating dmp for " << integration_time << "s at " << (int)frequency_Hz << "Hz" << endl;
  int n_time_steps = floor(frequency_Hz*integration_time);
  VectorXd ts = VectorXd::LinSpaced(n_time_steps,0,integration_time); // Time steps
  
  // Save trajectory 
  cout << "C++    |     Saving trajectory to file '" << traj_filename << "'"  << endl;
  DmpBbo::Trajectory trajectory;
  dmp->analyticalSolution(ts,trajectory);

  // Fixed impedance

  // Variable: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7360201

  // Trajectory set_misc converts 1xn_dims into n_time_stepsxn_nims
  MatrixXd impedance_matrix(1, 7);
  impedance_matrix << 1200, 1200, 1200, 600, 250, 250, 50;

  trajectory.set_misc(impedance_matrix);

  bool overwrite = true;    
  trajectory.saveToFile(traj_filename, overwrite);

  cout << "C++    |     Sending trajectory to client '" << endl;

  franka_tool_handover::JointAction::executeClient(trajectory);

  // FILE* file;
  // int argc;
  // char * argv[3];

  // argc = 2;
  // argv[0] = "create_cost_vars.py";
  // argv[1] = args[3];

  // Py_SetProgramName(argv[0]);
  // Py_Initialize();
  // PySys_SetArgv(argc, argv);
  // file = fopen("create_cost_vars.py","r");
  // PyRun_SimpleFile(file, "create_cost_vars.py");
  // Py_Finalize();

  delete dmp;
  
  return 0;
}