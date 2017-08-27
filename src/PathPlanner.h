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
  //============== Constants and Parameters =======================
  // time between two plannings
  const double INTERVAL = 0.02;
  // number of points in planned path
  const int PATH_LEN = 40;
  // speed in mph to m/s
  const double MPH2MPS = 0.447;
  // max speed of sdc
  const double MAX_SPEED = 20; // m/s
  // distance between front cars to maintain
  const double SAFE_CAR_DISTANCE = 20; //meters
  // max steps to use from pervious plan for current plan
  const size_t MAX_PLAN_LOOKBACK = 20;
  //============================ States =============================
  // indicator state whether the car is in the middle of lane changing
  bool in_lane_change;
  // s path from previous planning
  Points previous_s_path;

  //============== Member functions ===============================
  // car's stay in lane behavior

public:
  Path plan(const Path & previous_path,const SelfDrivingCar & sdc,
              const vector<PeerCar> &peers);

  Path keep_lane(const Path & previous_path, const SelfDrivingCar & sdc,const vector<PeerCar> & peers);
  Path change_lane(const Path & previous_path, const SelfDrivingCar & sdc,
  const vector<PeerCar> & peers,int lane_change);
  double accelerate(double current_speed, double target_speed) const;
  bool is_safe_to_change_lanes(const SelfDrivingCar & sdc, const vector<PeerCar> & peers, int target_lane) const;
};

#endif
