#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

// define used to leave out plotting sections from "production code" :)
#define PLOTDEBUG 0 // define as 1 to see plots for first PLOTSTEPS number of steps in simulation

#define PLOTSTEPS 600 // if PLOTDEBUG is defined as 1, simulation ends after this number of steps


#if PLOTDEBUG
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

using CppAD::AD;

// TODO: Set the timestep length and duration

// horizon set as 1second, this corresponds to ~27.8m for a car going at 100km/h
size_t N = 10;
double dt = 0.1;

#if PLOTDEBUG
static int counter = 0;
static   std::vector<double> x_vals = {};
static  std::vector<double> y_vals = {};
static  std::vector<double> psi_vals = {};
static  std::vector<double> v_vals = {};
static  std::vector<double> cte_vals = {};
static  std::vector<double> epsi_vals = {};
static std::vector<double> delta_vals = {};
static std::vector<double> a_vals = {};
#endif
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0; // target cross-track error
double ref_epsi = 0; // target heading error
double ref_v = 100 / 3.6; // target velocity in m/s

// starting positions of vectors in ADVector
// since the horizon has N members:
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// weights of costs. 
double weight_cte = 10;
double weight_epsi = 1000;
double weight_v = 1;
double weight_steer = 10;
double weight_acc = 5;
double weight_steer_jump = 1000;
double weight_acc_jump = 1;
double weight_steer_times_v = 30;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
    // calculation of cost. Based on course material (quiz solution)
    //------------------------------------------------------------
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += weight_cte * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += weight_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weight_steer * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weight_acc * CppAD::pow(vars[a_start + t], 2);
      fg[0] += weight_steer_times_v * CppAD::pow(vars[v_start + t] * vars[delta_start + t], 2);
      //simuate "Derivative Control" give penalty to rate of change of cte:
      //fg[0] += weight_cte  * 5 * CppAD::pow(vars[cte_start + t + 1], 2) - CppAD::pow(vars[cte_start + t], 2);
      //simuate "Derivative Control" give penalty to rate of change of epsi:
      //fg[0] += weight_epsi  * 5 *CppAD::pow(vars[epsi_start + t + 1],2) - CppAD::pow(vars[epsi_start + t], 2);
    }

     // Minimize the value gap between sequential actuations. (smoothness)
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weight_steer_jump * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weight_acc_jump * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    //---------------end calculation of cost-----------------------

    //------Set up constraints (based on quiz solution)------------
    // Initial constraints
    //
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
    for (int t = 0; t < N - 1; t++) {
      // The state at time t.
      AD<double> x0 = vars[x_start + t];
      AD<double> y0 = vars[y_start + t];
      AD<double> psi0 = vars[psi_start + t];
      AD<double> v0 = vars[v_start + t];
      AD<double> cte0 = vars[cte_start + t];
      AD<double> epsi0 = vars[epsi_start + t];

      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t + 1];
      AD<double> y1 = vars[y_start + t + 1];
      AD<double> psi1 = vars[psi_start + t + 1];
      AD<double> v1 = vars[v_start + t + 1];
      AD<double> cte1 = vars[cte_start + t + 1];
      AD<double> epsi1 = vars[epsi_start + t + 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t];
      AD<double> a0 = vars[a_start + t];
      
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }

    //------End set up constraints --------------------------------
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  #if PLOTDEBUG
  counter++;
  cout << "STEP: " << counter << endl;
  #endif
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N-1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  #if PLOTDEBUG
  x_vals.push_back(state[0]);
  y_vals.push_back(state[1]);
  psi_vals.push_back(state[2]);
  v_vals.push_back(state[3]);
  cte_vals.push_back(state[4]);
  epsi_vals.push_back(state[5]);
  #endif
  

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // lower & upper bounds for vars. Based on quiz solution
  //------------------------------------------------------
   // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  //cout << "setting vars bounds" << endl;
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  //cout << "setting delta bounds" << endl;
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332; // == -25 * pi /180
    vars_upperbound[i] = 0.436332;
  }

  //cout << "setting acc bounds" << endl;
  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  //cout << "setting constraints" << endl;
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
  
  //cout << "calling eval" << endl;
  //------------------------------------------------------
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  //cout << "eval returned" << endl;
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
  //options += "Numeric max_cpu_time          0.5\n";
  options += "Numeric max_cpu_time          50\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok)
  {
    cout << "solution not found" << endl;
    cout << "solution status: " << solution.status << endl;
    cout << "solution size: " << solution.x.size() << endl;
    cout << "vars :" << vars << endl;
    cout << "steering: " << solution.x[delta_start] << endl;
    cout << "throttle: " << solution.x[a_start] << endl;
    assert(ok); // quit application and see values in terminal
  }
  //cout << "solution.status : " << solution.status << endl;
  //cout << "Cappd success : " << CppAD::ipopt::solve_result<Dvector>::success << endl;
  //cout << "OK? : " << ok << endl;
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result = {solution.x[delta_start], solution.x[a_start],
            solution.x[x_start], solution.x[y_start],
            solution.x[x_start + 1], solution.x[y_start + 1],
            solution.x[x_start + 2], solution.x[y_start + 2],
            solution.x[x_start + 3], solution.x[y_start + 3],
            solution.x[x_start + 4], solution.x[y_start + 4],
            solution.x[x_start + 5], solution.x[y_start + 5],
            solution.x[x_start + 6], solution.x[y_start + 6]};


#if PLOTDEBUG
  delta_vals.push_back(solution.x[delta_start]);
  a_vals.push_back(solution.x[a_start]);
  if (cte > 2.5 || counter > PLOTSTEPS){
  plt::subplot(3, 2, 1);
  plt::title("CTE");
  plt::plot(cte_vals);

  plt::subplot(3, 2, 2);
  plt::title("EPSI");
  plt::plot(epsi_vals);

  plt::subplot(3, 2, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::subplot(3, 2, 4);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  
  plt::subplot(3, 2, 5);
  plt::title("throttle");
  plt::plot(a_vals);
  
  

  plt::show();
  assert(0);
  }
#endif
  return result;
}
