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
constexpr double pi() {return M_PI;}
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// road map
class Map {
public:

  Map(const Points & x,
      const Points & y,
      const Points & s,
      const Points & dx,
      const Points & dy):
      x(x), y(y), s(s), dx(dx), dy(dy) {

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
  virtual ~Map() {}

  // find the index of lane given the d coordinate of a car
  // lane starts with index 0 from the leftmost
  int find_lane(double car_d) const {
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
  int find_front_car_in_lane(const SelfDrivingCar & sdc,
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
  int find_rear_car_in_lane(const SelfDrivingCar & sdc,
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

public:
  Points x; // map coordinates
  Points y;
  Points s; // distance from starting point
  // vector orthogonal to road
  Points dx;
  Points dy;

  const double LANE_WIDTH = 4; /*meters*/
  const int RIGHTMOST_LANE = 2; /*lane 0, 1, 2*/
  const double MAX_S = 6945.554;

  // mapping from (s) -> (x, y) for each lane
  vector<Trajectory> lane_s2x;
  vector<Trajectory> lane_s2y;
};
