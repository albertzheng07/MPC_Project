#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

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

double ref_v = 15;

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
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.

    // Minimize reference error for t = 0 to t = N 
    for (int t = 0; t < N; t++) {
      fg[0] += 2000*CppAD::pow(vars[cte_start+t],2);
      fg[0] += 2000*CppAD::pow(vars[epsi_start+t],2);
      fg[0] += 100*CppAD::pow(vars[v_start+t]-ref_v,2);
    }  

    // Minimize actuator inputs for t = 0 to t = N - 1
    for (int t = 0; t < N-1; t++) {
      fg[0] += 4000*CppAD::pow(vars[delta_start+t],2);
      fg[0] += 1000*CppAD::pow(vars[a_start+t],2);
      fg[0] += 2000*CppAD::pow(vars[delta_start+t]*vars[a_start+t],2);
    }  

    // Minimize actuator rate inputs for t = 0 to t = N - 2
    for (int t = 0; t < N-2; t++) {
      fg[0] += 1000*CppAD::pow(vars[delta_start+t]-vars[delta_start+t+1],2);
      fg[0] += 1000*CppAD::pow(vars[a_start+t]-vars[a_start+t+1],2);
    } 

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

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
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 =   vars[v_start + t];
      AD<double> cte_1 = vars[cte_start + t];
      AD<double> epsi_1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];      
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte_0 = vars[cte_start + t - 1];
      AD<double> epsi_0 = vars[epsi_start + t - 1];
      AD<double> delta_0 = vars[delta_start + t - 1];
      AD<double> a_0 = vars[a_start + t - 1];

      AD<double> f_0 = 0.0;
      for (int i = 0; i < coeffs.size(); i++) {
        f_0 += coeffs[i] * CppAD::pow(x0, i); // f0 = p0 + p1*x0 + p2*x0**2 // y des
      }

      AD<double> psi_des = 0.0;
      for (int i = 1; i < coeffs.size(); i++) {
        psi_des += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0) = p1 + 2*p2*x0  // psi des
      }
      psi_des = CppAD::atan(psi_des);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // Add Model constraints, x(t+k+1) = A*x(t+k)+B*u(t+k)
      // Kinematic Bicycle Model with augmented states of CTE and Psi Error
      // xdot = x0 + cos(psi)*dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      // ydot = y0 + sin(psi)*dt
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // psidot = psi0 + v0/Lf*delta*dt
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf*delta_0*dt);
      // vdot = v0 + a*dt
      fg[1 + v_start + t] = v1 - (v0 + a_0*dt);
      // ctedot = xtrackError0 + xtrackErrorRate
      // CTE0 = ydes - y0 (current xtrack error)
      // CTERate = ydotError = v0*sin(epsi0)*dt
      fg[1 + cte_start + t] = cte_1 - ((f_0 - y0) + v0 * CppAD::sin((epsi_0)) * dt);      
      // epsidot = (psi_des - psi0)+v0/Lf*delta*dt  
      fg[1 + epsi_start + t] = epsi_1 - ((psi_des - psi0) - v0/Lf*delta_0*dt);     
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

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars =  N * 6 + (N - 1) * 2; // 6 states * N time steps, 2 actuators * N-1 actuations
  size_t n_constraints = 6*N; // 6 states * N time steps, constraints only on states

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // state variable limits set to arbitrary large value [x, y, psi, v, cte, psi]
  for (i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e4;
    vars_upperbound[i] = 1.0e4;
  }

  // steering value bounds delta min, delta max
  for (i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.436; // -25 deg
    vars_upperbound[i] = 0.436;  // 25 deg
  }

  // throttle value bounds a min, a max
  for (i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1; 
    vars_upperbound[i] = 1; 
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // initial limits for constraints
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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector <double> result;

  // Actuator solution
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);  

  // Vehicle trajectory solution at N steps 
  for (int i = 0; i < N; i++) {
    result.push_back(solution.x[x_start+i]);
    result.push_back(solution.x[y_start+i]);    
  }

  return result;  
}
