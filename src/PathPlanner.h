#ifndef PATHPLANNER_H
#define PATHPLANNER_H


#include "Map.h"

class PathPlanner  {
 public:

  PathPlanner(const Map & map);
  virtual ~PathPlanner();

private:
  Map map;

private:
  const double INTERVAL = 0.02; // sampling time
  const int PATH_LEN = 50; // total number of planning points
  const double MPH2MPS = 0.447; // speed in mph to m/s
  const double MAX_SPEED = 20; // m/s, max speed of ego vehicle, m/s
  const double SAFE_CAR_DISTANCE = 20; //meters, safet distance to maitain
  // max steps to use from pervious plan for current plan
  const size_t MAX_PLAN_LOOKBACK = 20;
  // indicator state whether the car is in the middle of lane changing
  bool in_lane_change;
  // s path from previous planning
  Points previous_s_path;


//---------------- member functions--------------------------
public:
  // behavior planning
  Path plan(const Path & previous_path,const EgoCarInfo  & ego,
              const vector<PeerCar> &peers);
  // generate path for keeping lane
  Path keep_lane(const Path & previous_path, const EgoCarInfo  & ego,const vector<PeerCar> & peers);
  // generate path for lane change scenario
  Path change_lane(const Path & previous_path, const EgoCarInfo  & ego,
  // surround vehicle info
  const vector<PeerCar> & peers,int lane_change);
  // accelerate strategy
  double accelerate(double current_speed, double target_speed) const;
  // lane change safet judgement
  bool is_safe_to_change_lanes(const EgoCarInfo  & ego, const vector<PeerCar> & peers, int target_lane) const;
};

#endif
