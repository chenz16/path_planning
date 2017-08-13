#ifndef PP_H
#define PP_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
//#include "helpfunc.cpp"

using namespace std;

#define DT 0.02

struct VehInfo {
	int id;
	double x;
	double y;
  double speed;
  double s;
  double d;
  double vx;
  double vy;
	double yaw;
};

struct PathInfo {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> d;
  vector<double> yaw;
  double x_end;
  double y_end;
  double s_end;
  double d_end;
  double yaw_end;
  size_t path_size;
};

class PP {
 public:

  PP();

  virtual ~PP();

  size_t path_size = 50;
  double V_ref = 50;
  VehInfo EgoVeh_info;
  VehInfo FrontVeh_info;
  vector<vector<double>> other_vehicles;


  PathInfo NewPath;
  PathInfo PrevPath;


  void RetrievePreviousPathInfo(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> veh_info);
  //void init_env(vector<double> maps_s, vector<double> maps_s, vector<double> maps_d);
  void update_env(vector<vector<double>> other_vehicles);
  void FindFront();
  void ResetNewPath();
  void RetainPathXY();
  void generate_path_straight();
  void generate_path_circle();
  vector<double> generate_path_MPC();
  void update_path(vector<vector<double>> xy_path);

};

#endif /* MPC_H */
