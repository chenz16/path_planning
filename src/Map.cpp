#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "Map.h"

Map::Map(const Points & x,const Points & y,const Points & s,
    const Points & dx, const Points & dy): x(x), y(y), s(s), dx(dx), dy(dy) {

  auto n = x.size();
  assert (x.size() == n);
  assert (y.size() == n);
  assert (s.size() == n);
  assert (dx.size() == n);
  assert (dy.size() == n);

  double HALF_LANE = LANE_WIDTH / 2;
  // initialize gitter - adding gitter for numerical stability
  double lower = -0.5;
  double upper = 0.1;
  uniform_real_distribution<double> unif(lower, upper);
  default_random_engine re(random_device{}());
  // Calculate trajectory for each lane based on waypoints
  for (int lane = 0; lane <= RIGHTMOST_LANE; ++lane) {
    vector<double> lane_x;
    vector<double> lane_y;
    vector<double> lane_s;
    for (auto i = 0; i < n; ++i) {
      double jitter = -0.2;//unif(re);
      // map to middle of lane
      lane_x.push_back(x[i] + dx[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
      lane_y.push_back(y[i] + dy[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
      lane_s.push_back(s[i]);
    }
    // this is a quick fix to make it work when car goes back to starting point
    // a more realistic solution will be to fit local models segment by segment
    for (auto i = 0; i < n; ++i) {
      double jitter = -0.2;//unif(re);
      lane_x.push_back(x[i] + dx[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
      lane_y.push_back(y[i] + dy[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
      lane_s.push_back(s[i] + MAX_S);
    }
    Trajectory s2x; s2x.set_points(lane_s, lane_x);
    Trajectory s2y; s2y.set_points(lane_s, lane_y);

    lane_s2x.push_back(s2x);
    lane_s2y.push_back(s2y);
  }

}
Map::~Map() {}

// find the index of lane given the d coordinate of a car
// lane starts with index 0 from the leftmost
int Map::find_lane(double car_d) const {
  int lane = floor(car_d / LANE_WIDTH);
  // assert ((lane >= 0) and (lane <= RIGHTMOST_LANE));
  // some cars may have invalid d value at the beginning
  lane = max(0, lane);
  lane = min(RIGHTMOST_LANE, lane);
  return lane;
}

// find the index of a peer car immediately before self-driving-car
// on a certain lane
// return -1 if there is no car found
int Map::find_front_car_in_lane(const SelfDrivingCar & sdc,
                           const vector<PeerCar> & peer_cars,
                           int lane) const {
  int found_car = -1;
  double found_dist = INF;
  for (auto i = 0; i < peer_cars.size(); ++i) {
    const PeerCar & car = peer_cars[i];
    auto car_lane = find_lane(car.d);
    if ( (car_lane == lane) and (car.s >= sdc.s) ) {
      double dist = car.s - sdc.s;
      if (dist < found_dist) {
        found_car = i;
        found_dist = dist;
      }
    }
  }
  return found_car;
}

// find the index of a peer car immediately after self-driving-car
// on a certain lane
// return -1 if there is no car found
int Map::find_rear_car_in_lane(const SelfDrivingCar & sdc,
                          const vector<PeerCar> & peer_cars,
                          int lane) const {
  int found_car = -1;
  double found_dist = INF;
  for (auto i = 0; i < peer_cars.size(); ++i) {
    const PeerCar & car = peer_cars[i];
    auto car_lane = find_lane(car.d);
    if ( (car_lane == lane) and (car.s <= sdc.s) ) {
      double dist = sdc.s - car.s;
      if (dist < found_dist) {
        found_car = i;
        found_dist = dist;
      }
    }
  }
  return found_car;
}
