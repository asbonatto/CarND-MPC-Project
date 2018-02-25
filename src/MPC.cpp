#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

double ref_v = 125.0*0.44704;
const double Lf = 2.67;
double dt = 1.5*fmax(Lf/ref_v, 1.0*(100*1E-3)); 
size_t N = 10;


vector<size_t> get_idxs(int state_size, int control_size, int N){
    // Auxiliary function to get the indices
    
    vector<size_t> idxs;
    size_t idx = 0;
    
    idxs.push_back(idx);
    
    for (int i = 1; i < (state_size + control_size - 1); i++){
        idx += N;
        idxs.push_back(idx);
    }
    idx += N - 1;
    idxs.push_back(idx);
    
    return idxs;
    
}

class FG_eval {
    public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs; 
    }
    
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        
        size_t t;
        // ----------------------------------------
        // Define cost
        fg[0] = 0;
        
        vector<size_t> idxs = get_idxs(6, 2, N);
        // Cost part related to the reference trajectory
        for (t = 0; t < N; t++) {
            // Remember that state is  x, y, psi, v, cte, epsi, d, a
            fg[0] += 1*CppAD::pow(vars[idxs[3] + t] - ref_v, 2);
            fg[0] += 50*CppAD::pow(vars[idxs[4] + t], 2);
            fg[0] += 10000*CppAD::pow(vars[idxs[5] + t], 2);

        }
        
        // Penalize high actuations to avoid saturation
        for (t = 0; t < N - 1; t++) {
            // Remember that state is  x, y, psi, v, cte, epsi, d, a
            fg[0] += 500*CppAD::pow(vars[idxs[6] + t], 2);
            fg[0] += 25*CppAD::pow(vars[idxs[7] + t], 2);
        }
        
        // Penalize discontinuity
        for (t = 0; t < N - 2; t++) {
            // Remember that state is  x, y, psi, v, cte, epsi, d, a
            fg[0] += 2000*CppAD::pow(vars[idxs[6] + t + 1] - vars[idxs[6] + t], 2);
            fg[0] += 25*CppAD::pow(vars[idxs[7] + t + 1] - vars[idxs[7] + t], 2);
        }
        
        // ----------------------------------------
        // Define the constraints

        // Get initial states
        for (int i = 0; i < 6; i++){
            fg[1 + idxs[i]] = vars[idxs[i]];
        }
        
        for (t = 1; t < N; t++) {
            /* 
                As done in the quizzes, I'm transfering the values for 
                readability
            */
            
            // The state at time t+1 .
            AD<double> x1 = vars[idxs[0] + t];
            AD<double> y1 = vars[idxs[1] + t];
            AD<double> psi1 = vars[idxs[2] + t];
            AD<double> v1 = vars[idxs[3] + t];
            AD<double> cte1 = vars[idxs[4] + t];
            AD<double> epsi1 = vars[idxs[5] + t];
            
            // The state at time t.
            AD<double> x0 = vars[idxs[0] + t - 1];
            AD<double> y0 = vars[idxs[1] + t - 1];
            AD<double> psi0 = vars[idxs[2] + t - 1];
            AD<double> v0 = vars[idxs[3] + t - 1];
            AD<double> cte0 = vars[idxs[4] + t - 1];
            AD<double> epsi0 = vars[idxs[5] + t - 1];
            
            // Only consider the actuation at time t.
            AD<double> delta0 = vars[idxs[6] + t - 1];
            AD<double> a0 = vars[idxs[7] + t - 1];
            
            AD<double> f0 = coeffs[0];
            for (int i = 1; i < coeffs.size(); i++){
                f0 += coeffs[i]*CppAD::pow(x0, i);
            }
            
            AD<double> psides0 = coeffs[1];
            for (int i = 2; i < coeffs.size(); i++){
                psides0 += i*coeffs[i]*CppAD::pow(x0, i - 1);
            }
            psides0 = CppAD::atan(psides0);
            
            // Kinematic equations
            fg[1 + idxs[0] + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + idxs[1] + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + idxs[2] + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[1 + idxs[3] + t] = v1 - (v0 + a0 * dt);
            fg[1 + idxs[4] + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + idxs[5] + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
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
    
    size_t n_vars = state.size() * N + 2 * (N - 1);
    size_t n_constraints = N * state.size(); // State-space dimension
    
    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }
    
    vector<size_t> idxs = get_idxs(state.size(), 2, N);
    for (int i = 0; i < state.size(); i++){
        vars[idxs[i]] = state[i];
    }
    
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // State variables do not have an upper limit, so setting up 
    // an arbitrary value
    for (i = 0; i < idxs[state.size()]; i++) {
        vars_lowerbound[i] = -std::numeric_limits<double>::max();
        vars_upperbound[i] = +std::numeric_limits<double>::max();
    }
    // delta : [-25; 25] deg.
    for (i = idxs[state.size()]; i < idxs[state.size() + 1]; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = +0.436332;
    }
    // a : [-1; 1]
    for (i = idxs[state.size() + 1]; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = +1.0;
    }
    
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    
    for (int i = 0; i < state.size(); i++){
        constraints_lowerbound[idxs[i]] = state[i];
        constraints_upperbound[idxs[i]] = state[i];
    }
    
    // object that computes objective and constraints
    FG_eval fg_eval(coeffs); // ref_v, N
    
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
                                          options, vars
                                        , vars_lowerbound, vars_upperbound
                                        , constraints_lowerbound
                                        , constraints_upperbound
                                        , fg_eval, solution);
    
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok){
        std::cout << "WARN : optmizer error" << endl;
    }
    
    // Cost
    // auto cost = solution.obj_value;
    
    // Returning the controls and the planned trajectory
    vector<double> output;
    output.push_back(solution.x[idxs[6]]);
    output.push_back(solution.x[idxs[7]]);
    
    for (size_t t = 1; t < N - 1; t++) {
        output.push_back(solution.x[idxs[0] + t]);
        output.push_back(solution.x[idxs[1] + t]);
    }
    
    return output;
}

