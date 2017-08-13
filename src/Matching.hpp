#ifndef MATCHING_H
#define MATCHING_H

#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
using namespace std;


using namespace std;

class Matching {
public:

 Matching();

 virtual ~Matching();

 int map_size;
 int indx_CurrentPos;
 double len_map;
 vector<double> polyfit_waypoints;
 vector<double> map_waypoints_s;

 void init_MapWaypoints(map_waypoints_s);
 int find_PosIndx(double s);
 vector<double> indentify_SubMapWaypoints(int indx, laps);
 vector<double> generate_path(vector<double> polyfit_waypoints, ds);
 vector<double> getXY(s, d);


}
