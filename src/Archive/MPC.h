#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;



#define W_col 9
#define W_jerk 10.0
#define W_v 0.01
#define ref_v  50*1.6/3.6

class MPC {
 public:

   vector<double> mpc_x;
   size_t N;
   size_t x_start=0;
   size_t v_start=x_start+N;
   size_t cte_start=v_start+N;
   size_t a_start=cte_start+N;

   MPC();
   virtual ~MPC();
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void set_para(size_t N);
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
