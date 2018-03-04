#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
    double v_ref, Lf, dt, a_ref;
    size_t N;
    vector<double> weights;
    
 public:
  MPC(double v_ref, double Lf, size_t N, double latency);
  
  void set_weights(vector<double> weights);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
