#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <fstream>
#include "utils.hpp"

// for convenience
using json = nlohmann::json;

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    // Create log file for tracking steering and throttle values, to check their behaviour
   // fstream file(logFile(), ios::out | ios::binary);
   // file << "Steering Throttle" << endl;

    vector<double> prev_state = {0.0, 0.0, 0.0, 0.0};


    h.onMessage([&mpc,/* &file,*/ &prev_state](uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

        double latency = 0.1;

        vector<double> weights = {0.2, 10.0, 0.1, 4500.0, 1.0, 10.0, 0.0};

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double x = j[1]["x"];
                    double y = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steer_angle = j[1]["steering_angle"];  // steering angle is in the opposite direction
                    double throttle = j[1]["throttle"];

                    v = mph2ms(v);
                    double acceleration = calculate_acceleration(prev_state, v, x, y);

                    //Fit road coordinates into polynom
                    auto coeffs = roadCoeffs(x, y, psi, ptsx, ptsy);

                    // Prepare state vector in car coordniates
                    Eigen::VectorXd state(8); // {x, y, psi, v, cte, epsi, steer_angle, acceleration}

                    double cte = polyeval(coeffs, 0); // cross treck error
                    double epsi = -atan(polyevalDer1(coeffs, 0));  //orientation error
                    // x, y, psi = 0  because we are in the car coordinate system
                    state << 0, 0, 0, v, cte, epsi, steer_angle, acceleration;

                    // call MPC solver
                    auto vars = mpc.Solve(state, coeffs, latency, weights);

                    double steer_value =  vars[0] / (deg2rad(25));
                    double throttle_value = vars[1];


                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                   // file << steer_value << " " << throttle_value << endl;


                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for (size_t i = 2; i < vars.size(); i = i + 2) { //the first two are steer angle and throttle value
                        mpc_x_vals.push_back(vars[i]);
                        mpc_y_vals.push_back(vars[i + 1]);
                    }
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


                    for (int i = 0; i < 25; i++) { // index staring from 1 for visualize
                        // only the reference point which is in the front of the car
                        next_x_vals.push_back(i*5);
                        next_y_vals.push_back(polyeval(coeffs, i*5));//ptsy_car[i]);
                    }

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //  std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds((int)(latency * 1000)));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h/*, &file*/](uWS::WebSocket <uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        //Close log file on disconnect
       // file.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
