#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;


// For convinience, some constant and functions are defined here
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
size_t N = 200;
const double dt = 0.08;
const double delta = deg2rad(2.0);
const double a = 0.5;


/**
	Given the trajectory and its corresponding steering configuration from simulation.
	To identify the vehicle parameter Lf, the identification task is reformed 
	to a optimization problem. The difference between predicted trajectory using kinematic
	model and the actual trajectory is minimized.
	200 time step is used to create trajectory with 200 points, the decision variables are the states
	variable (x, y, psi, v) of these 200 points and Lf, so totally 801 variables. Therefore, using the
	kinematic model, we can formulate 800 constraints. 
	
*/

// define the index for convinience, since all the variables are concatenated to 1 vector

int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int v_start = psi_start + N;
int Lf_start = v_start + N;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
		the fg fector is used to evaluate the constraints and cost function
		vars is the decision variable vector
     */
	 
	 // store the cost in fg[0]
	 fg[0] = 0;
	 
	 // costs regards reference trajectory
	 for ( int i = 0; i < N; ++i) {
		 
		 fg[0] += CppAD::pow(CppAD::sqrt(CppAD::pow(vars[x_start+i]-coeffs[0],2) + 
		 CppAD::pow(vars[y_start+i]-coeffs[1],2))- coeffs[2], 2);
	 }
	 
	 // Reform the kinematic model to constraints of optimization problem:
	 // x_t+1 = x_t + vx * dt ==> fg[t] = x[t+1] - (x[t] + vx * dt) == 0 
	 // constraint on decision variables on first time step equals the given initial state
	 // Since cost is stored at the first index, all index is increased by 1
	 fg[1 + x_start] = vars[x_start];
	 fg[1 + y_start] = vars[y_start];
	 fg[1 + psi_start] = vars[psi_start];
	 fg[1 + v_start] = vars[v_start];

	 for ( int i = 1; i < N; ++i) {
		 // the next states
		 AD<double> x1 = vars[x_start + i];
		 AD<double> y1 = vars[y_start + i];
		 AD<double> psi1 = vars[psi_start + i];
		 AD<double> v1 = vars[v_start + i];
		 
		 
		 // the current states
		 AD<double> x0 = vars[x_start + i - 1];
		 AD<double> y0 = vars[y_start + i - 1];
		 AD<double> psi0 = vars[psi_start + i - 1];
		 AD<double> v0 = vars[v_start + i - 1];
		 AD<double> Lf = vars[Lf_start];

		 // implement the transition function
		 // x_t+1 = x_t + v * cos(psi) * dt
		 // y_t+1 = y_t + v * sin(psi) * dt
		 // psi_t+1 = psi_t + v / Lf * delta_t * dt
		 // v_t+1 = v_t + throttle_t * dt
		 // cte_t+1 = f(x_t) - y_t + v * sin(epsi_t) * dt
		 // epsi_t+1 = psides_t - psi_t + v_t / Lf * delta_t * dt
		 
		 fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		 fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		 fg[1 + psi_start + i] = psi1 - (psi0 + v0 * delta / Lf * dt);
		 fg[1 + v_start + i] = v1 - (v0 + a * dt);
	 } 
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Set the number of model variables (decision variables of the optimization problem).
   * For example: If the state is a 4 element vector, the parameter to be identified is a 
   *   scalar and there are 10 timesteps. The number of variables is then:
   *   4 * 10 + 1
   */
  size_t n_vars = 4 * N + 1;
  
  /**
   * Set the number of constraints
   */
  size_t n_constraints = 4 * N;

  // Initial value of the independent variables.
  // The initialization is based on the kinematic model and the intial guess of Lf:Lf_0
  // This model-based initialization will make the solution of optimization problem easier
  
  Dvector vars(n_vars);
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[Lf_start] = state[4];
  
  for (size_t i = 1; i < N; ++i) {
    vars[x_start+i] = vars[x_start+i-1] + vars[v_start+i-1] * cos(vars[psi_start+i-1]) * dt;
	vars[y_start+i] = vars[y_start+i-1] + vars[v_start+i-1] * sin(vars[psi_start+i-1]) * dt;
	vars[psi_start+i] = vars[psi_start+i-1] + vars[v_start+i-1] / vars[Lf_start] * delta * dt;
	vars[v_start+i] = vars[v_start+i-1] + a * dt;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  /**
   * Set lower and upper limits for variables. 
   */
  
  for (int i = 0; i < y_start; ++i) {
	  vars_lowerbound[i] = 50.0;
	  vars_upperbound[i] = 400.0;
  }
  
  for (int i = y_start; i < psi_start; ++i) {
	  vars_lowerbound[i] = 100;
	  vars_upperbound[i] = 575;
  }
  for (int i = psi_start; i < v_start; ++i) {
	  vars_lowerbound[i] = 0;
	  vars_upperbound[i] = 20*pi();
  }
    for (int i = v_start; i < Lf_start; ++i) {
	  vars_lowerbound[i] = -5;
	  vars_upperbound[i] = 200;
  }
  
  
  vars_lowerbound[Lf_start] = 1.0;
  vars_upperbound[Lf_start] = 5.0;

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = state[0];
  constraints_upperbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_upperbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_upperbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_upperbound[v_start] = state[3];


  // object that evaluate objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Options for detailed print information
  options += "Integer print_level  5\n";
  
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  
  // As offline application, the maximum CPU time is set to 100 to ensure the solution can be found 
  options += "Numeric max_cpu_time          100\n";

  // object to sore solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * Return the estimated Lf and the corresponding x,y coordinates for visualization of the result
   */

  std::vector<double> vars_out(4*N+1);
  
  for (int i = x_start; i < x_start+N; ++i) {
	  vars_out[i] = solution.x[i];
  }
  for (int i = y_start; i < y_start + N; ++i) {
	  vars_out[i] = solution.x[i];
  }
  for (int i = psi_start; i < psi_start + N; ++i) {
	  vars_out[i] = solution.x[i];
  }
  for (int i = v_start; i < v_start + N; ++i) {
	  vars_out[i] = solution.x[i];
  }
  
  vars_out[4*N] = solution.x[Lf_start];

  return vars_out;
}
