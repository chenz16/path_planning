#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
using namespace std;




Matching::Matching() {}
Matching::~Matching() {}

void Matching::init_MapWaypoints(vector<double> map_waypoints_s){
    this-> map_waypoints_s = map_waypoints_s;
    map_size = map_waypoints_s.size();
    len_map  = map_waypoints_s[map_size-1];
    laps = 0;
}

int Matching::find_PosIndx(double s){
  if (s<=0){
      indx_CurrentPos = 0;
    }
   else if (s>= len_map){
      indx_CurrentPos = len_map;
    }
    else {
    for (int i =0; i<map_size -1; i++) {
      if (s >= map_waypoints_s[i] & s<= map_waypoints_s[i+1]){
      indx_CurrentPos = i;
      }
    }
   }
 }

 
