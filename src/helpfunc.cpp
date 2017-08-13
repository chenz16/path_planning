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

int FindFront(double s, double d, vector<vector<double>> other_vehicles){
  int indx=0 ;
  int N_veh = other_vehicles.size();
  double dis_front = 10000;
  for (int i =0; i< N_veh; i++){
     double s_diff = other_vehicles[i][5] - s;
     double d_diff = d - other_vehicles[i][6];
     if (abs(d_diff)<2 && s_diff >=0 && s_diff <= 50 && s_diff < dis_front){
        indx = i;
     }

  }
  return indx;

}






            /*
            double dist_inc = 0.5;
            for(int i = 0; i < 2; i++)
            {
              next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
              next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            } */

            /*double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();

            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            if(path_size == 0)
            {
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            }
            else
            {
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                double pos_x2 = previous_path_x[path_size-2];
                double pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            } */


            /*int ind_nearest = FindFront(car_s, car_d, sensor_fusion);
            double dist_inc = 0;
            double dis_near = sensor_fusion[ind_nearest][5];
            dis_near = dis_near - car_s;
            double vx_near = sensor_fusion[ind_nearest][3];
            double vy_near = sensor_fusion[ind_nearest][4];
            double vs_near = sqrt(vx_near*vx_near + vy_near*vy_near);
            cout<<"vs_near"<<vs_near<<" dis_near"<<dis_near<<endl;

            if ((dis_near <= 50.0) && (dis_near >=0)){
              dist_inc = (vs_near-7) * 0.02;
            } else {
              dist_inc = (50-7)*0.02;
            } */

            //dist_inc = 0.2;
            /*for(int i = 0; i < 50-path_size; i++)
            {
                pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));

                next_x_vals.push_back(pos_x);
                next_y_vals.push_back(pos_y);
                //cout<<pos_x<<"-"<<pos_y<<endl;

            }*/

            /*cout<<"end path s ="<< end_path_s  <<"path size = "<< path_size<<endl;
            cout<<"dis_near="<< dis_near<<endl;
            double pos_s ;
            //double dist_inc=0.5;
            int timer = 0;
            timer += 1;
             if (timer<2) {
              pos_s = car_s + dist_inc*(path_size-1);
            } else {pos_s= end_path_s;}*/

            //double car_front_speed = vx_near;
            //double car_front_s = sensor_fusion[ind_nearest][5];
            //double car_front_d = sensor_fusion[ind_nearest][6];
             //vector<double> s_traj = car_following(car_s, car_speed, car_front_s,car_front_speed);


          /*  for (int i = 0; i < 50- path_size; i++)
            {
              pos_s += dist_inc;
              auto xy = getXY(pos_s, 10, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              cout<<xy[0]<<"-"<<xy[1]<<endl;
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }*/
