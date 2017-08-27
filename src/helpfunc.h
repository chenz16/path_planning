#ifndef HELPFUNC_H
#define HELPFUNC_H

#include <fstream>
#include <iostream>
#include <vector>
#include "spline.h"

using namespace std;
/**************************Constants and Types*******************/
const double INF = numeric_limits<double>::infinity();
typedef vector<double> Points;
typedef vector<double> SensorData;
typedef tk::spline Trajectory;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
string hasData(string s);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

// ego vehicle struct
struct EgoCarInfo {
public:
  double x; // based on map coordinate
  double y;
  double s; //based on frenet coordinate
  double d;
  double yaw;
  double speed; // vehicle speed
  EgoCarInfo (double x, double y, double s, double d, double yaw, double speed);//constrcutor
  virtual ~EgoCarInfo ();
};


// other vehicle info
struct PeerCar {
public:
  int id;
  double x; // based on map coordinate
  double y;
  double vx; // velocity
  double vy;
  double s; // based on frenet coordinate
  double d;
  PeerCar(const SensorData & sensor);
  virtual ~PeerCar();
};

// path struct
struct Path {
public:
  Points xs;// map coordinate
  Points ys;
  size_t size() const;
  Path(const Points xs, const Points ys);
  Path (const Path & rhs);
  virtual ~Path();
};

#endif
