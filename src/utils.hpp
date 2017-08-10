#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <chrono>

using namespace std;

#ifndef MPC_UTILS_H
#define MPC_UTILS_H
// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
inline string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
inline double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * std::pow(x, i);
    }
    return result;
}

inline double polyevalDer1(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += coeffs[i] * i * std::pow(x, i - 1);
    }
    return result;
}

inline double polyevalDer2(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 2; i < coeffs.size(); i++) {
        result += coeffs[i] * i * (i - 1) * std::pow(x, i - 2);
    }
    return result;
}



// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// convert from map coordinate to car coordinates
inline void map2car(double x, double y, double psi, const std::vector<double>& ptsx_map, const std::vector<double>& ptsy_map,
             Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car){

    for(std::size_t i=0; i < ptsx_map.size(); i++){
        double dx = ptsx_map[i] - x;
        double dy = ptsy_map[i] - y;
        ptsx_car[i] = dx * cos(0-psi) - dy * sin(0-psi);
        ptsy_car[i] = dx * sin(0-psi) + dy * cos(0-psi);
    }
}

inline string logFile(){
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"%Y-%m-%d--%I-%M-%S",now);
    return "../logs/steering_throttle" + string(buffer) + ".txt";
}

inline double mph2ms(double v){
    return v * 0.44704;
}

inline double calculate_acceleration(vector<double> &prev_state, double v, double x, double y){
    double dt = 0;
    double acceleration = 0;

    auto now = std::chrono::system_clock::now().time_since_epoch();
    double now_s = now.count() / 1e9;

    dt = now_s - prev_state[0];

    double estimatedDistance;
    double distance;

    estimatedDistance = (v + prev_state[1])/2*dt;
    distance = sqrt(pow(prev_state[2] - x, 2) + pow(prev_state[3] - y, 2));

    cout << "Estimated: " << estimatedDistance << " Actual: " << distance << " Dt: "<< dt << endl;

    if(prev_state[0] == 0){
        prev_state[0] =  now_s;
        prev_state[1] = v;
    } else {
        prev_state[0] =  now_s;
        acceleration = (v - prev_state[1])/dt;
        prev_state[1] = v;
    }


    prev_state[2] = x;
    prev_state[3] = y;

    return  acceleration;
}

inline Eigen::VectorXd roadCoeffs(double x, double y, double psi, const std::vector<double>& ptsx, const std::vector<double>& ptsy){

    // Convert from global coordinate to car coordinate
    Eigen::VectorXd ptsx_car(ptsx.size());
    Eigen::VectorXd ptsy_car(ptsy.size());
    map2car(x, y, psi, ptsx, ptsy, ptsx_car, ptsy_car);

    // Fit middle lane to polynom. Calculate polynom coefficients
    auto coeffs = polyfit(ptsx_car, ptsy_car, 3); // 3rd order line fitting
    return coeffs;
}
#endif //MPC_UTILS_H
