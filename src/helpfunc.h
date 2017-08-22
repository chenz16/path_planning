#ifndef HELPFUNC_H
#define HELPFUNC_H

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
using namespace std;
using json = nlohmann::json;

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

// Self driving car
struct SelfDrivingCar {
public:

  SelfDrivingCar(double x, double y,
                 double s, double d,
                 double yaw, double speed):
                 x(x), y(y),
                 s(s), d(d),
                 yaw(yaw),
                 speed(speed) {};

  double x; // map coordinate
  double y;
  double s; // frenet coordinate
  double d;
  double yaw; // bearing
  double speed; // speed
};

// other cars on the road
struct PeerCar {
public:
  PeerCar(const SensorData & sensor):
    id(sensor[0]), x(sensor[1]), y(sensor[2]),
    vx(sensor[3]), vy(sensor[4]),
    s(sensor[5]), d(sensor[6]) {}

  int id;
  double x; // map coordinate
  double y;
  double vx; // velocity
  double vy;
  double s; // frenet coordinate
  double d;
};



class Map(const Points & x, const Points & y,const Points & s, const Points & dx,const Points & dy)
{

}
