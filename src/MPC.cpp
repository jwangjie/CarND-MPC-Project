#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.12;

// set the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// set the reference velocity
double ref_v = 80;
// set the reference cross track error and orientation error 
// even they all 0
double ref_cte = 0.0;
double ref_epsi = 0.0; 

// set weights to each component of the cost function 
// be tuned based on the relative importance of each part
double weight_cet = 1500.0;
double weight_epsi = 1000.0;
double weight_constant_vel = 1.0;
double weight_delta = 20.0;
double weight_a = 40.0;
double weight_delta_diff = 40.0;
double weight_a_diff = 80.0;

// The IPOPT solver takes all the state variables and actuator
// variables in a singular vector. Thus, variables were established 
// in a way that one variable was starts and another ends.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost function and vehicle model/constraints.
  // `vars` is a vector containing the variable values used by the cost funciton 
  // and model(state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Cost function 
    // Define the cost related the reference state and
    // any anything you think may be beneficial.
    // The part of the cost based on the reference state.

    /*
    Since 0 is the index at whichIpopt expects fg to store the cost value, 
    we sum all the components of the cost and store them at index 0.

    In each iteration through the loop, we sum three components to reach the 
    aggregate cost: our cross-track error, our heading error, and our velocity error.
    */
    for (int t = 0; t < N; t++) {
      fg[0] += weight_cet * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += weight_constant_vel * CppAD::pow(vars[v_start + t] - ref_v, 2); // avoid stopping
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weight_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weight_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weight_delta_diff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weight_a_diff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup model constraints
    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // 3rd order polynomial fitting 
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 *x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /*
  Take of the plant latency problem 
  */
  // retrive states [x,y,ψ,v,cte,eψ] and actutations [δ,a] from Eigen::VectorXd state
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  // set the actuations
  double delta = 0.0;
  double a = 0.0;

  // solving the latency problem by predicting states (current + latency) using process model
  // before sending these states to solver
  double latency_dt = 0.1;
  double x_new = x + v * cos(psi) * latency_dt;
  double y_new = y + v * sin(psi) * latency_dt;
  double psi_new = psi + v/Lf * delta * latency_dt;
  double v_new = v + a * latency_dt;
  double cte_new = cte + v * sin(epsi) * latency_dt;
  double epsi_new = epsi + v/Lf * delta * latency_dt;

  // send the new states without latency to the solver
  x = x_new;
  y = y_new;
  psi = psi_new;
  v = v_new;
  cte = cte_new;
  epsi = epsi_new;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // set the upper and lower limits (-pi/3, pi/3) of the psi
  // to avoid U turn of the car, meaning to search
  // gloabl optimization instead of local minima
  // This is NOT necessary for this project but a good point
  // to consider for future similar project
  for (int i = psi_start; i < v_start; i++) {
    vars_lowerbound[i] = -1.047197;
    vars_upperbound[i] = 1.047197;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
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

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  std::vector<double> results;

  results.push_back(solution.x[delta_start]);
  results.push_back(solution.x[a_start]);

  // the MPC predicted path x and y position
  for (int i = 0; i < N - 1; i++){
    results.push_back(solution.x[x_start + i + 1]);
    results.push_back(solution.x[y_start + i + 1]);
  }

  //return controls (delta, a) and path (x, y)
  return results; 

}
