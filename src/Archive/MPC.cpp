#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// TODO: Set the timestep length and duration
double dt = 0.02;

/*size_t x_start = 0;
size_t v_start = x_start + N;
size_t cte_start = v_start + N;
size_t a_start = cte_start + N;*/

class FG_eval {
 public:
  // Fitted polynomial coefficients
  size_t N;
  size_t x_start=0;
  size_t v_start=x_start+N;
  size_t cte_start=v_start+N;
  size_t a_start=cte_start+N;
  Eigen::VectorXd coeffs;

  //FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  FG_eval(Eigen::VectorXd coeffs, size_t N) {
    this->coeffs = coeffs;
    this->N = N;}
    //std::cout<<N<<std::endl;}
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
   void operator()(ADvector& fg, const ADvector& vars) {
      fg[0] = 0;
      for (int t = 0; t < N; t++) {
        fg[0] += W_v*CppAD::pow(vars[v_start + t] - ref_v, 2); // speed track
        //g[0] += W_col*CppAD::pow(vars[v_start + t]/vars[cte_start + t], 2); // collosion

      }

      for (int t = 0; t < N - 2; t++) {
        fg[0] += W_jerk * CppAD::pow(vars[a_start+t+1]-vars[a_start+t],2); // jerk
      }

      fg[1+x_start] = vars[x_start];
      fg[1+v_start] = vars[v_start];
      fg[1+cte_start] = vars[cte_start];
      //fg[1+a_start]  = vars[a_start];

      for (int t = 1; t < N; t++) {
          AD<double> x1 = vars[x_start+t];
          AD<double> v1 = vars[v_start+t];
          AD<double> cte1 = vars[cte_start+t];

          AD<double> x0 = vars[x_start+t-1];
          AD<double> v0 = vars[v_start+t-1];
          AD<double> cte0 = vars[cte_start+t-1];

          AD<double> a0 = vars[a_start + t -1];

          AD<double> f0 = 0;

          fg[1+x_start +t] = x1 - (x0+v0*dt + a0*CppAD::pow(dt,2)/2.0);
          fg[1+v_start +t] = v1 - (v0 + a0*dt);
          fg[1+cte_start +t] = cte1 - cte0 - (dt*coeffs[1] - (v0*dt + a0*CppAD::pow(dt,2)/2.0) + 3*a0*dt);
          //fg[1+cte_start +t] = coeff[0] + dt*t*coeffs[1] - (x1 + 0.5*v1);
          // add here for jerk max constraints
     }
     // add here for jerk max constraints

     for (int t = 0; t < N-2; t++) {
       AD<double> a1 = vars[a_start + t+1];
       AD<double> a0 = vars[a_start + t];
       fg[a_start+t] = (a1-a0)/dt;
     }


  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

  void MPC::set_para(size_t N) {
      this->N = N;
     }

  vector<double>  MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

   double x =   state[0];
   double v =   state[1];
   double cte = state[2];

  size_t n_vars = N*3 + (N-1)*1;
  // TODO: Set the number of constraints
  // size_t n_constraints = N * 6;
  size_t n_constraints = N * 3 + (N-2)*1;
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
 cout<<"n_satart="<<n_vars<<"\n"<<endl;
  /*for (int i = 0; i < v_start; i++){
     vars_lowerbound[i] = 0;
     vars_upperbound[i] = 1.0e5;
  } // for x

  for (int i = v_start; i < cte_start; i++){
     vars_lowerbound[i] = 0;
     vars_upperbound[i] = 26.7;
  } // for v

  for (int i = cte_start; i < a_start; i++){
     vars_lowerbound[i] = 10;
     vars_upperbound[i] = 1.0e6;
  } // for cte


  for (int i = a_start; i < n_vars; i++){
     vars_lowerbound[i] = -2;
     vars_upperbound[i] = 2;
  }// for a


  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // equality constraint
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

// initial value constraints (equality)
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = coeffs[0] - x - 3*v;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = coeffs[0] - x -3*v;
//inequality constraints
  for (int i = a_start; i < n_constraints; i++) {
    constraints_lowerbound[i]=-3;
    constraints_upperbound[i]=3;
  } */
  //for (int i = cte_start+1; i < N; i++) {
    //constraints_lowerbound[i]=0;
    //constraints_upperbound[i]=1.0e5;
  //}
  // object that computes objective and constraints

  /*FG_eval fg_eval(coeffs, this->N);

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
     this->mpc_x = {}; */

  /*for (int i = 0; i < N; i++) {
      this->mpc_x.push_back(solution.x[x_start + i]);
        //this->mpc_y.push_back(solution.x[y_start + i]);}
      //return {    solution.x[x_start + 1 ],   solution.x[v_start + 1],
        //          solution.x[a_start + 1], solution.x[cte_start + 1]};
      }*/
  return {1.0, 2.0};


}
