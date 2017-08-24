#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "helpfunc.h"
#include "spline.h"

using Points = vector<double>;
using Trajectory = tk::spline;


class Map (const Points & x, const Points & y, const Points & s, const Points & dx, const Points & dy){
public:
  Points x;
  Points y;
  Points s;
  Points dx;
  Points dy;
  const double LANE_WIDTH = 4; /*meters*/
  const int RIGHTMOST_LANE = 2; /*lane 0, 1, 2*/
  const double MAX_S = 6945.554;
  // mapping from (s) -> (x, y) for each lane
  vector<Trajectory> lane_s2x;
  vector<Trajectory> lane_s2y;

  Map();
  virtual ~ Map();

  int find_lane(double car_d) const;
  int find_front_car_in_lane(const SelfDrivingCar & sdc, const vector<PeerCar> & peer_cars,
                             int lane) const;
  int find_rear_car_in_lane(const SelfDrivingCar & sdc,
                            const vector<PeerCar> & peer_cars,
                            int lane) const;

  }


class PathPlanner (const Map & map) {
 public:

  PathPlanner();

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
  const double SAFE_CAR_DISTANCE = 15; //meters
  // max steps to use from pervious plan for current plan
  const size_t MAX_PLAN_LOOKBACK = 20;
  //============================ States =============================
  // indicator state whether the car is in the middle of lane changing
  bool in_lane_change;
  // s path from previous planning
  Points previous_s_path;

  //============== Helper functions ===============================
  // car's stay in lane behavior
  Path keep_lane(const Path & previous_path, const SelfDrivingCar & sdc,
                const vector<PeerCar> & peers);
  Path change_lane(const Path & previous_path, const SelfDrivingCar & sdc,
  const vector<PeerCar> & peers,int lane_change);
  double accelerate(double current_speed, double target_speed);
  bool is_safe_to_change_lanes(const SelfDrivingCar & sdc, const vector<PeerCar> & peers, int target_lane);
}
