//#include "MPC.h"
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <chrono>
#include "PP.h"
#include "MPC.h"
#include "helpfunc.cpp"


using namespace std;
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    PP pp;
    MPC mpc;
    bool veh_start = false;


    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            vector<double> veh_info = {car_x, car_y, car_s, car_d, car_yaw, car_speed};
            pp.RetrievePreviousPathInfo(previous_path_x, previous_path_y, veh_info);
            pp.update_env(sensor_fusion);
            //pp.generate_path_circle();
            pp.FindFront();
          Eigen::VectorXd coeffs(2);
          Eigen::VectorXd state(3);
          coeffs << pp.FrontVeh_info.s, pp.FrontVeh_info.speed;

          double s = pp.EgoVeh_info.s;
          double speed = pp.EgoVeh_info.speed;
          double cte = pp.FrontVeh_info.s - pp.EgoVeh_info.s;

          state <<s, speed, cte;
          //vector<double> mpc_path=mpc.Solve(state, coeffs);
          //vector<double> start_path;
          vector<double> s_path (50);
          //vector<double> ss_path;

        //if (veh_start==false) {
            for (int i=0; i< 50; i++)
            {
              s_path[i]=car_s +2.0*i*0.02;
            }
          /*  if (car_speed > 1) {veh_start = true;}

        } else { s_path = mpc.Solve(state, coeffs);}*/

            //if (veh_start==false && car_speed<0.5)
            //{ s_path = start_path;
              //veh_start = true;}
            //else {s_path = mpc_path;}

// MPC
              size_t N = 15;
              mpc.set_para(15);
              //cout<<mpc.v_start<<endl;
              vector<double> ss_path=mpc.Solve(state, coeffs);



          cout<<"veh_start" << veh_start<<endl;

          vector<vector<double>> xy_path(2, vector<double>(pp.NewPath.path_size));
          for (int i=0; i< pp.NewPath.path_size; i++){
              vector<double> xy_grid = getXY(s_path[i], 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              xy_path[0][i] = xy_grid[0];
              xy_path[1][i] = xy_grid[1];
            }

            pp.update_path(xy_path);
            //next_x_vals  = pp.NewPath.x;
            //next_y_vals  = pp.NewPath.y;
            next_x_vals = xy_path[0];
            next_y_vals  =xy_path[1];


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
