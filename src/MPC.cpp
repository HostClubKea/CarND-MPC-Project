#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <chrono>
#include <iostream>
#include "utils.hpp"

using namespace std;
using namespace std::chrono;

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.2;

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

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


// Reference velocity
double ref_v = 80 * 0.447; // convert from mph to m/s
double ref_v_original = 120 * 0.447;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    vector<double> weights;

    FG_eval(Eigen::VectorXd coeffs, vector<double> weights) {
        this->coeffs = coeffs;
        this->weights = weights;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (size_t t = 0; t < N; t++) {
            fg[0] += weights[0]*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += weights[1]*CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += weights[2]*CppAD::pow(vars[v_start + t] - ref_v, 2);
        }
        // Minimize the use of actuators.
        for (size_t t = 0; t < N - 1; t++) {
            fg[0] += weights[3]*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += weights[4]*CppAD::pow(vars[a_start + t], 2);
        }
        // Minimize the value gap between sequential actuations.
        for (size_t t = 0; t < N - 2; t++) {
            fg[0] += weights[5]*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += weights[6]*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];


        // Other constraints
        // Other constraints
        for (std::size_t t = 1; t < N; t++) {
            // State at time t + 1
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];

            // State at time t
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];

            // Actuator constraints at time t only
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
            AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*CppAD::pow(x0,2));

            // Setting up the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

double total = 0;
double n = 0;

Eigen::VectorXd MPC::Latency(double latency, Eigen::VectorXd state,  Eigen::VectorXd coeffs) {
    double x    = state[0];
    double y    = state[1];
    double psi  = state[2];
    double v    = state[3];
    double cte  = state[4];
    double epsi = state[5];
    double steer_angle = state[6];
    double acceleration = state[7];

    double real_latency = latency;

    if(n!= 0)
        real_latency += total/n;

    x = 0.0 + v * real_latency;
    y = 0.0;
    psi = 0.0 + v * (-steer_angle) / Lf * real_latency;
    cte = cte + v * sin(epsi) * latency;
    epsi = epsi + v * (-steer_angle) / Lf * real_latency;
    v = v + acceleration * real_latency;

    state << x, y, psi, v, cte, epsi, steer_angle, acceleration;
    return state;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double latency, vector<double> weights) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    //Our controller have latency,
    //We can deal with it by predicting our state after latency
   if(latency > 0){
        state = Latency(latency, state, coeffs);
   }


    double x    = state[0];
    double y    = state[1];
    double psi  = state[2];
    double v    = state[3];
    double cte  = state[4];
    double epsi = state[5];
    double steer  = state[6];
    double acc = state[7];

    // Calculate radius of curvature
    double rx = 20;
    double radius = abs(pow(1 + pow(polyevalDer1(coeffs, rx), 2), 1.5)/ polyevalDer2(coeffs, rx));

    if(radius < 50){
        ref_v = ref_v_original / 1.7;
    } else if(radius < 60) {
        ref_v = ref_v_original / 1.5;
    } else {
        ref_v = ref_v_original;
    }

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2;
    // TODO: Set the number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start]    = x;
    vars[y_start]    = y;
    vars[psi_start]  = psi;
    vars[v_start]    = v;
    vars[cte_start]  = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // TODO: Set lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 3.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;
    //constraints_lowerbound[a_start] = acc;
    //constraints_lowerbound[delta_start] = steer;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
    //constraints_lowerbound[a_start] = acc;
    //constraints_lowerbound[delta_start] = steer;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs, weights);

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
    CppAD::ipopt::solve_result <Dvector> solution;


    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>( t2 - t1 ).count()/1000000.0;
    total += duration;
    n++;

    cout << "Current: " << duration << " Average: " << total/n << endl;

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
//    return {solution.x[x_start + 1],   solution.x[y_start + 1],
//            solution.x[psi_start + 1], solution.x[v_start + 1],
//            solution.x[cte_start + 1], solution.x[epsi_start + 1],
//            solution.x[delta_start],   solution.x[a_start]};
    // Return the first actuator values
    double steering = solution.x[delta_start];
    double acceleration = solution.x[a_start];
    vector<double> ouputs = {steering, acceleration};

    // attach the predicted route to display
    for (i=0; i<N; i++) {
        ouputs.push_back(solution.x[x_start+i]);
        ouputs.push_back(solution.x[y_start+i]);
    }

    return ouputs;
}
