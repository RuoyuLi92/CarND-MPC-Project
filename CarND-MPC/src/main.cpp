#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  // Prediction horizon 

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
		  // recieve the reference trajectory and current trajectory from simulator
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
 	  /*
          Eigen::VectorXd vxd_ptsx = Eigen::VectorXd::Map(&ptsx[0],ptsx.size());
                  Eigen::VectorXd vxd_ptsy = Eigen::VectorXd::Map(&ptsy[0],ptsy.size());
		  auto coeffs = polyfit(vxd_ptsx, vxd_ptsy, 3);
		  double cte = polyeval(coeffs, px) - py;
		  double epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px);
		  
		  VectorXd state(6);
		  
		  state << px, py, psi, v, cte, epsi;
		  
		  auto vars = mpc.Solve(state, coeffs);
                  std::cout<<vars.size();
	  */  
          for (size_t i = 0; i < ptsx.size(); ++i) {
              double x_shift = ptsx[i] - px;
              double y_shift = ptsy[i] - py;

              ptsx[i] = x_shift * cos(psi) + y_shift * sin(psi);
              ptsy[i] = x_shift * sin(-psi) + y_shift * cos(psi);
          }
          /*
          VectorXd ptsx_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(),ptsx.size());
          VectorXd ptsy_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(),ptsy.size());
          */
          Eigen::Map<Eigen::VectorXd> ptsx_(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_(&ptsy[0], ptsy.size());

          auto coeffs = polyfit(ptsx_, ptsy_, 3);
          // calculated in vehicle coordinate, and vhicle position is always the origin point
          // so cte = f(0) - 0;
          // epsi = psi - atan(f'(0));
          double cte = polyeval(coeffs, 0.0);
          double epsi = -atan(coeffs[1]);

          VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          
          auto vars = mpc.Solve(state, coeffs);

          int L = 10;
          std::cout << "before get output" << std::endl;

          double steer_value = -1 * vars[2*L - 2] / deg2rad(25);
          double throttle_value = vars[2*L - 1];
          std::cout << "after get output" << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals(L-1);
          vector<double> mpc_y_vals(L-1);
          std::cout << "after intialization" << std::endl;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
		  
		  for (int i = 0; i < L-1; ++i) {
              mpc_x_vals[i] = vars[i];
			  mpc_y_vals[i] = vars[i+L-1]; 
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          std::cout << "after set mpc"<< std::endl;

          // Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_.size());
          vector<double> next_y_vals(ptsy_.size());
          std::cout<<"after intiliaze reference"<<std::endl;
          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
		   
		  for (int i = 0; i < ptsx_.size(); ++i) {
			  next_x_vals[i] = ptsx_[i];
			  next_y_vals[i] = ptsy_[i]; 
		  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(0));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
