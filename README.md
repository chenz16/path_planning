# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Go through the Rubric

### The code compiles correctly:

The code compiles correctly but with a few warnings. I was not able to resolve those warnings. Looks for any suggestion how to remove the warnings.

### The car is able to drive at least 4.32 miles without incident.

The car is able to at least run 4.32 mile without incident. However, after the first lap, the car MAY suddenly run out of track and after a few seconds, it returns back. Did not have time to find the root cause.

### The car drives according to the speed limit.

No violation to speed

### Max Acceleration and Jerk are not Exceeded.

No violation

### Car does not have collisions.

No collision.

### The car stays in its lane, except for the time between changing lanes.

The car stays in its lane except for the time during lane change.

### The car is able to change lanes

The car is able to change lanes as expected.

### There is a reflection on how to generate paths.

I defined a class called PathPlanner which plans lane change behavior and also generate the path:

        Path plan(const Path & previous_path,const EgoCarInfo  & ego,const vector<PeerCar> &peers);
        // generate path for keeping lane
        Path keep_lane(const Path & previous_path, const EgoCarInfo  & ego,const vector<PeerCar> & peers);
        // generate path for lane change scenario
        Path change_lane(const Path & previous_path, const EgoCarInfo  & ego,  const vector<PeerCar> &peers)

In the behavior planning step,  current ego vehicle information `EgoCarInfo  & ego `, the surrounding vehicle information `const vector<PeerCar> &peers`, and previous planned path  `const Path & previous_path` are used to determine either the ego vehicle wants to keep the current lane or want to do lane change. If it is the former, it plans the path based on function call `Path keep_lane`; if it is the latter, it plans the path based on lane change method `Path change_lane`. More details are described as follows:

#### Preprocess real-time information before path planning:  

- Align the vector size of previous path vector "s" and "x/y" by removing what has been consumed by the controller:

		auto n_consumed = previous_s_path.size() - previous_path.size();
		previous_s_path.erase(previous_s_path.begin(), previous_s_path.begin() + n_consumed);
  

- Do not plan new plath during lane change unless the length of remaining path length from previous step is less than desired path length. 

	    if (in_lane_change) {
	      if (previous_path.size() <= PATH_LEN) {
		in_lane_change = false;
	      }
	      return previous_path;
	    }


- Collect the ego vehicle and surrounding vehicle information. 

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

####  Make decision for keeping or changing lane. After the decision is made, call the motion planning strategy to plan the path accordingly. 

-  if the front vehicle speed is low, consider changing lane;

		if (frontcar_speeds[ego_lane] <= 0.95 * MAX_SPEED) { ....}

-  find a suitable lane based on current lane, speed difference and distance difference compared with the front vehicle at the target lane. 


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
		      bool is_target_better = (frontcar_speeds[target_lane] >= frontcar_speeds[ego_lane] * 1.1);
		      is_target_better &= (frontcar_dists[target_lane] + 30 >= frontcar_dists[ego_lane]);
		      // or there is bigger room on the target lane to bypass the current traffic
		      is_target_better |= (frontcar_dists[target_lane] >= 100 + frontcar_dists[ego_lane]);
		      if (is_target_better) {
			return change_lane(previous_path, ego, peers, target_lane - ego_lane);
		      }
		    

-  plan the path either for keeping lane or changing lane through function call:

            if (frontcar_speeds[ego_lane] <= 0.95 * MAX_SPEED) {
              .....
              return change_lane(previous_path, ego, peers, target_lane - ego_lane);
             .....}
              return keep_lane(previous_path, ego, peers);



#### How to keep lane: 

- A car following strategy is developed in the function"PathPlanner::keep_lane". This strategy looks at the distance of ego vehicle from the front vehicle. If it is too close (less than the safe distance), it will set current vehicle speed target as a percentage  of front vehicle speed, else it is set at the maximum target speed; The car acceleration is proportional to the speed difference of ego vehicle and the front target speed. 

	    double speed = last_speed;
	    for (auto i = newplan_start; i < PATH_LEN; ++i) {
	      speed = accelerate(speed, target_speed);
	      s_path.push_back(s_path.back() + speed * INTERVAL);
	    }

-  This lane keeping strategy also does special handling for the first move in order to move the vehicle. 

		double last_speed = -1; // last speed from previous plan
		int newplan_start  = -1; // step to start the new plan
		if (! is_previous_plan_available) {
		s_path.push_back(ego.s + 0.005); // first move
		last_speed = ego.speed * MPH2MPS;
		newplan_start = 1;
		}  


- The new path reuses remaining points of previous path up to s_path.size() steps;

      auto n = previous_s_path.size();
      for (auto i = 0; i < min(MAX_PLAN_LOOKBACK, n); ++i) {
        s_path.push_back(previous_s_path[i]);
      }
      last_speed = (s_path.back() - s_path[s_path.size()-2]) / INTERVAL;
      newplan_start = s_path.size();



#### How to plan lane change:

- first, check if it is safe to change lane:

		if (! is_safe_to_change_lanes(ego, peers, target_lane)) {
		// keep in lane if the dynamic env changes
		return keep_lane(previous_path, ego, peers);
		} else { .....


- Partial of the way points of new path come from current lane (by looking back for smoothness) and partial depends on the new lane. There are some way points gap between current lane point and future lane points for smoothness. After the grid points of new path are obtained, use spline function to smooth these path points. All of the path planning is first done in Frenet  coordination system and then it is converted to map coordinate system through the spine functions whose grid points are obtained from global maps. 

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

### Summary 

The planned path is able to run on the track in the simulator for one lap successfully. However it is far from the real situation!!

