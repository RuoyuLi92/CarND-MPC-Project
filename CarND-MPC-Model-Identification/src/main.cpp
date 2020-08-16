#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"
#include <fstream>
#include <sstream>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::endl;
using std::ofstream;
using std::ios;
using std::cout;

int main() {

  const int L = 200;
  
  int x_start = 0;
  int y_start = x_start + L;
  int psi_start = y_start + L;
  int v_start = psi_start + L;
  int Lf_start = v_start + L;
  
  // Initial state from reference trajectory
  const double x_0 = 200.0;
  const double y_0 = 370.0;
  const double psi_0 = 3.14159;
  const double v_0 = 0.0;
  
  // Initial guess of parameter to be estimated
  const double Lf_0 = 1.0;
  
  // Coefficients of reference trajectory
  const double center_x = 200.644;
  const double center_y = 287.718;
  const double radius = 81.862;
  
  VectorXd state(5);
  state << x_0, y_0, psi_0, v_0, Lf_0;
  
  VectorXd coeffs(3);
  coeffs << center_x, center_y, radius;
  
  // Estimater is initiliazed here
  MPC estimator;
  
  auto vars = estimator.Solve(state, coeffs);
  
  ofstream xout("value_output.txt", ios::app);
  xout << "x_out:" << endl; 
  for (size_t i = 0; i < L; ++i) {
	  ofstream xout("value_output.txt", ios::app);
	  xout << vars[i] << " ";
  }
  xout << endl << "y_out:" << endl;
  
  for (size_t i = y_start; i < y_start + L; ++i) {
	  ofstream xout("value_output.txt", ios::app);
	  xout << vars[i] << " ";
  }

  xout << endl << "psi_out:" << endl;
  
  for (size_t i = psi_start; i < psi_start + L; ++i) {
	  ofstream xout("value_output.txt", ios::app);
	  xout << vars[i] << " ";
  }

  xout << endl << "v_out:" << endl;
  
  for (size_t i = v_start; i < v_start + L; ++i) {
	  ofstream xout("value_output.txt", ios::app);
	  xout << vars[i] << " ";
  }
  xout << endl << "Lf:" << vars[4*L];
  
  cout << "Lf estimated:" << vars[4*L] << endl;
  
  return 0;
}

