#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

using Eigen::VectorXd;

const int latency = 100;
const double Lf = 2.67;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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
double polyeval(VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals,
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

int main() {
    uWS::Hub h;
    
    std::ifstream fparam("../src/param.json");
    json mpc_param;
    fparam >> mpc_param;
    
    // MPC is initialized here!
    MPC mpc(mpc_param["Constants"]["v_ref"] , mpc_param["Constants"]["Lf"], mpc_param["Constants"]["N"], double(latency));
    
    vector<double> weights;
    weights.push_back(mpc_param["Weights"]["wv"]);
    weights.push_back(mpc_param["Weights"]["wcte"]);
    weights.push_back(mpc_param["Weights"]["wepsi"]);
    weights.push_back(mpc_param["Weights"]["wdelta"]);
    weights.push_back(mpc_param["Weights"]["wa"]);
    weights.push_back(mpc_param["Weights"]["wjump_delta"]);
    weights.push_back(mpc_param["Weights"]["wjump_a"]);
    mpc.set_weights(weights);
    
    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
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
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steer_value = j[1]["steering_angle"];
                    double throttle_value = j[1]["throttle"];
                    
                    VectorXd wpts_x = VectorXd::Constant(ptsx.size(), 0.0);
                    VectorXd wpts_y = VectorXd::Constant(ptsy.size(), 0.0);
                    
                    // Converting to navigation coordinates
                    for (size_t i = 0; i < ptsx.size(); i++) {
                        double dx = ptsx[i] - px;
                        double dy = ptsy[i] - py;
                        wpts_x[i] = (+dx * cos(psi) + dy * sin(psi));
                        wpts_y[i] = (-dx * sin(psi) + dy * cos(psi));
                    }
                    
                    // Trajectory polynomial and tracking errors
                    auto coeffs = polyfit(wpts_x, wpts_y, 3);
                    
                    double dt = double(latency)/1000;
                    steer_value *= -1.0; // Coordinate transformation
                    v *= 0.44704; // Converting to m/s
                    px  = dt*v*cos(steer_value);
                    py  = dt*v*sin(steer_value);
                    psi = dt*v/Lf*sin(steer_value);
                    
                    double cte  = polyeval(coeffs, px) - py;
                    double epsi = psi - atan(coeffs[1]);
                    v+= throttle_value*dt;
                    
                    
                    // Create the state vector
                    VectorXd state = VectorXd::Constant(6, 0.0);
                    
                    state << px, py, psi, v, cte, epsi;
                    
                    /*
                        NOTE : 
                        does performance improve if we make te dt and N 
                        adaptive, i.e, more resolution when v and/or curvature are
                        higher?
                    */
                    
                    vector<double> output = mpc.Solve(state, coeffs); 
                    steer_value    = output[0]/deg2rad(25);
                    throttle_value = output[1];
                    
                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    
                    //Display the MPC calculated trajectory 
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (size_t i = 1; i < (output.size()-2)/2; i++) {
                        
                        mpc_x_vals.push_back(output[2*i]);
                        mpc_y_vals.push_back(output[2*i + 1]);
                        
                    }
                    
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    for (int i = 0; i < wpts_x.size(); i++) {
                        next_x_vals.push_back(wpts_x[i]);
                        next_y_vals.push_back(wpts_y[i]);
                    }
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    this_thread::sleep_for(chrono::milliseconds(latency));
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
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
    size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
            } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length) {
        ws.close();
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
