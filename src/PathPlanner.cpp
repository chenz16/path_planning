#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "PathPlanner.h"
using namespace std;

PathPlanner::PathPlanner(const Map & map): map(map)
 {in_lane_change = false;}

PathPlanner::~PathPlanner() {}

  // plan path
Path PathPlanner::plan(const Path & previous_path,
            const EgoCarInfo  & ego,
            const vector<PeerCar> &peers) {

    // update previous path info by removing what has been consumed by controller
    auto n_consumed = previous_s_path.size() - previous_path.size();
    previous_s_path.erase(previous_s_path.begin(), previous_s_path.begin() + n_consumed);

    // if in the middle of lane changing, finish it first
    if (in_lane_change) {
      // reserve the last part for next move
      if (previous_path.size() <= PATH_LEN) {
        // it is finished!
        in_lane_change = false;
      }
      // continue lane changing
      return previous_path;
    }

    // behavior strategy - keep / change lanes

    // collect road information
    int ego_lane = map.find_lane(ego.d);
    vector<double> frontcar_speeds;
    vector<double> frontcar_dists;
    for (auto lane = 0; lane <= map.RIGHTMOST_LANE; ++lane) {
      auto frontcar_idx = map.find_front_car_in_lane(ego, peers, lane);
      // default value if no front car found
      double frontcar_speed = INF;
      double frontcar_dist = INF;
      if (frontcar_idx != -1) {
        const PeerCar & frontcar(peers[frontcar_idx]);
        frontcar_speed = sqrt(frontcar.vx*frontcar.vx + frontcar.vy*frontcar.vy);
        frontcar_dist = frontcar.s - ego.s;
      }
      frontcar_speeds.push_back(frontcar_speed);
      frontcar_dists.push_back(frontcar_dist);
    }
    // make decision for keeping or changing lane
    // a. if speed expection not met, consider possibility to change lane
    // b. if a suitable lane is found meeting safety criteria, then change the lane
    //    else stay in lane

    // Consider changing lanes when speed is slow
    if (frontcar_speeds[ego_lane] <= 0.95 * MAX_SPEED) {

      // pick a target lane - only one lane crossing at a time is allowed
      int target_lane = ego_lane;
      if (ego_lane == 0) target_lane = 1;
      else if (ego_lane == 2) target_lane = 1;
      else /*ego_lane == 1*/ {
        // prefer changing from 1 -> 0
        target_lane = 0;
        // unless lane 2 is faster and safe to change to
        if ((frontcar_speeds[2] > frontcar_speeds[0]) and
              is_safe_to_change_lanes(ego, peers, 2))
          target_lane = 2; // changing from 1 -> 2
        // or can bypass
        if (frontcar_dists[2] >= frontcar_dists[0] + 100)
          target_lane = 2;
      }
      //cout << "Considering changing from " << ego_lane << " to " << target_lane << endl;
      // check if target lane is better then current lane
      // higher speed on target lane and there is enough room to fit in the target lane
      bool is_target_better = (frontcar_speeds[target_lane] >= frontcar_speeds[ego_lane] * 1.1);
      is_target_better &= (frontcar_dists[target_lane] + 30 >= frontcar_dists[ego_lane]);
      // or there is bigger room on the target lane to bypass the current traffic
      is_target_better |= (frontcar_dists[target_lane] >= 100 + frontcar_dists[ego_lane]);
      if (is_target_better) {
        return change_lane(previous_path, ego, peers, target_lane - ego_lane);
      }
    }

    // Keep lanes
    return keep_lane(previous_path, ego, peers);
  };

  // car's stay in lane behavior
  Path PathPlanner::keep_lane(const Path & previous_path,
    const EgoCarInfo  & ego,
    const vector<PeerCar> & peers) {

    // planning starts with path for s coordinates,
    // path for (x,y) will be derived based on it
    Points s_path;

    // collect information
    int ego_lane = map.find_lane(ego.d);
    int frontcar_idx = map.find_front_car_in_lane(ego, peers, ego_lane);

    // there are two conditions influencing the behaviors of keep-in-lane

    // whether there is a front car on the lane, it decides the target speed
    // of ego
    bool is_free_to_go = true;
    if (frontcar_idx != -1) {
      const PeerCar & car(peers[frontcar_idx]);
      double response_time = INTERVAL * PATH_LEN;
      // assume the front car can brake all of sudden
      auto safe_dist = ego.speed * MPH2MPS * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - ego.s;
      is_free_to_go = (dist >= safe_dist);
    }
    double target_speed = MAX_SPEED;
    if (! is_free_to_go ) {
      const PeerCar & car(peers[frontcar_idx]);
      double carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      target_speed = carspeed * 0.7;
    }
    // whether there is a previous plan (if not, it means the car just starts)
    // it decides whether to follow previous path for smooth driving
    bool is_previous_plan_available = (previous_path.size() >= 2);
    double last_speed = -1; // last speed from previous plan
    int newplan_start  = -1; // step to start the new plan
    if (! is_previous_plan_available) {
      s_path.push_back(ego.s + 0.005); // first move
      last_speed = ego.speed * MPH2MPS;
      newplan_start = 1;
    } else {
      auto n = previous_s_path.size();
      for (auto i = 0; i < min(MAX_PLAN_LOOKBACK, n); ++i) {
        s_path.push_back(previous_s_path[i]);
      }
      last_speed = (s_path.back() - s_path[s_path.size()-2]) / INTERVAL;
      newplan_start = s_path.size();
    }
    // continue the new plan
    double speed = last_speed;
    for (auto i = newplan_start; i < PATH_LEN; ++i) {
      speed = accelerate(speed, target_speed);
      s_path.push_back(s_path.back() + speed * INTERVAL);
    }

    // convert s path to (x, y) path
    Points x_path;
    Points y_path;
    // trajectory mapping from s -> (x, y)
    const auto & s2x(map.lane_s2x[ego_lane]);
    const auto & s2y(map.lane_s2y[ego_lane]);
    //const auto & s2x = map.lane_s2x[ego_lane];
    //const auto & s2y = map.lane_s2y[ego_lane];

    for (const auto & s: s_path) {
      x_path.push_back(s2x(s));
      y_path.push_back(s2y(s));
    }

    // remember the last plan
    previous_s_path = s_path;

    // return the path
    return Path(x_path, y_path);
  }

  // car's change lane behavior
  Path PathPlanner::change_lane(const Path & previous_path,
    const EgoCarInfo  & ego,
    const vector<PeerCar> & peers,
    int lane_change) {

    // collect information
    int ego_lane = map.find_lane(ego.d);
    int target_lane = ego_lane + lane_change;
    int target_frontcar_idx = map.find_front_car_in_lane(ego, peers, target_lane);
    int target_rearcar_idx = map.find_rear_car_in_lane(ego, peers, target_lane);
    int current_frontcar_idx = map.find_front_car_in_lane(ego, peers, ego_lane);

    if (! is_safe_to_change_lanes(ego, peers, target_lane)) {
      // keep in lane if the dynamic env changes
      return keep_lane(previous_path, ego, peers);
    } else {
      //cout << "finished changing lanes ..." << endl;
      // build trajectory for lane changing, which is
      // a smooth mapping from s -> (x, y) across lanes

      // current and target lanes trajectory
      const auto & current_s2x{map.lane_s2x[ego_lane]};
      const auto & current_s2y{map.lane_s2y[ego_lane]};
      const auto & target_s2x{map.lane_s2x[target_lane]};
      const auto & target_s2y{map.lane_s2y[target_lane]};
      // lane change trajectory
      Trajectory lanechange_s2x;
      Trajectory lanechange_s2y;
      //spline lanechange_s2x;
      //spline lanechange_s2y;

      // build up way points for lane change trajectory
      Points lanechange_s;
      Points lanechange_x;
      Points lanechange_y;
      double start_s = ego.s - 20; // on old lane
      double change_s = ego.s + 15; // across at this step
      if (target_frontcar_idx != -1) // careful with front car on target
        change_s = min(change_s, peers[target_frontcar_idx].s);
      double end_s = change_s + 100; // to the new lane
      for (auto s = start_s; s <= change_s; ++s) {
        lanechange_s.push_back(s);
        lanechange_x.push_back(current_s2x(s));
        lanechange_y.push_back(current_s2y(s));
      }
      // 50 points for crossing the lane
      for (auto s = change_s + 50; s <= end_s; ++s) {
        lanechange_s.push_back(s);
        lanechange_x.push_back(target_s2x(s));
        lanechange_y.push_back(target_s2y(s));
      }
      lanechange_s2x.set_points(lanechange_s, lanechange_x);
      lanechange_s2y.set_points(lanechange_s, lanechange_y);

      // build the plan based on lane change trajectory
      vector<double> s_path;
      // pick the previous path
      bool is_previous_plan_available = (previous_path.size() >= 2);
      double last_speed = -1; // last speed from previous plan
      int newplan_start  = -1; // step to start the new plan
      if (! is_previous_plan_available) {
        s_path.push_back(ego.s + 0.25); // first move
        last_speed = ego.speed * MPH2MPS;
        newplan_start = 1;
      } else {
        auto n = previous_s_path.size();
        for (auto i = 0; i < min(MAX_PLAN_LOOKBACK, n); ++i) {
          s_path.push_back(previous_s_path[i]);
        }
        last_speed = (s_path.back() - s_path[s_path.size()-2]) / INTERVAL;
        newplan_start = s_path.size();
      }
      // continue to build new path - it is a longer plan
      // than usual because it usually takes longer time
      // to change lanes
      double target_speed = min(last_speed * 1.02, MAX_SPEED);
      double speed = last_speed;
      for (auto i = newplan_start; i < PATH_LEN * 6; ++i) {
        speed = accelerate(speed, target_speed);
        s_path.push_back(s_path.back() + speed * INTERVAL);
      }

      // convert s path to (x, y) path
      Points x_path;
      Points y_path;
      for (const auto & s: s_path) {
        x_path.push_back(lanechange_s2x(s));
        y_path.push_back(lanechange_s2y(s));
      }

      // remember the last plan
      previous_s_path = s_path;

      // return the path
      in_lane_change = true;
      return Path(x_path, y_path);
    }
  }

  // car's modify speed bevhaivor - maintain acceleration and jerk constraints
  double PathPlanner::accelerate(double current_speed, double target_speed) const {
    double diff = target_speed - current_speed;
    double delta = diff * 0.005;
    if (fabs(diff) <= 0.01) delta = 0;
    return current_speed + delta;
  }

  // if it is safe to change lane based on road condition
  bool PathPlanner::is_safe_to_change_lanes(const EgoCarInfo  & ego,
                               const vector<PeerCar> & peers,
                               int target_lane) const {
    // Check the speed and distances of three peer cars wrt ego
    // namely "front car on current lane", "front and back cars on target lane"

    // collect road information
    int ego_lane = map.find_lane(ego.d);
    int target_frontcar_idx = map.find_front_car_in_lane(ego, peers, target_lane);
    int target_rearcar_idx = map.find_rear_car_in_lane(ego, peers, target_lane);
    int current_frontcar_idx = map.find_front_car_in_lane(ego, peers, ego_lane);

    // lanes must be valid
    bool is_safe_to_change = (target_lane >= 0) and (target_lane <= map.RIGHTMOST_LANE);
    // if the speed too slow, not safe to change lane within limited time
    is_safe_to_change &= (ego.s >= 100) and (ego.s <= 6900); // dont bother at beginning or end
    // check front car on target
    if (target_frontcar_idx != -1) {
      const PeerCar & car(peers[target_frontcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy + car.vy);
      double response_time = 4; //seconds
      auto safe_distance = fabs(ego.speed * MPH2MPS - carspeed) * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - ego.s;
      is_safe_to_change &= (dist >= safe_distance);
      /*if (dist < safe_distance)
        cout << "Safe to change? dist to target_front=" << dist << " safe_dist=" << safe_distance << endl;*/
    }
    // check rear car on target
    if (target_rearcar_idx != -1) {
      const PeerCar & car(peers[target_rearcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      double response_time = 4; //seconds
      auto safe_distance = fabs(carspeed - ego.speed*MPH2MPS) * response_time + SAFE_CAR_DISTANCE;
      auto dist = ego.s - car.s;
      is_safe_to_change &= (dist >= safe_distance);
    /*  if (dist < safe_distance)
        cout << "Safe to change? dist to target_rear=" << dist << " safe_dist=" << safe_distance << endl;*/
    }
    // check front car on current lane
    if (current_frontcar_idx != -1) {
      const PeerCar & car(peers[current_frontcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      double response_time = 3.0; //seconds
      auto safe_distance = fabs(ego.speed*MPH2MPS - carspeed) * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - ego.s;
      is_safe_to_change &= (dist >= safe_distance);
      /*if (dist < safe_distance)
        cout << "Safe to change? dist to current_front=" << dist << " safe_dist=" << safe_distance << endl;*/
    }

    return is_safe_to_change;
  };
