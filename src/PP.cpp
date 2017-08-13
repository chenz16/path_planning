#include "PP.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
//#include "helpfunc.cpp"
#include "MPC.h"


PP::PP() {}
PP::~PP() {}

void PP::RetrievePreviousPathInfo(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> veh_info)
{
  // copy previous path and get the path size
  PrevPath.x = previous_path_x;
  PrevPath.y = previous_path_y;
  PrevPath.path_size = previous_path_x.size();
  EgoVeh_info.x = veh_info[0];
  EgoVeh_info.y = veh_info[1];
  EgoVeh_info.s = veh_info[2];
  EgoVeh_info.d = veh_info[3];
  EgoVeh_info.yaw = veh_info[4]/180.0*M_PI;
  EgoVeh_info.speed = veh_info[5];

  // get retrieve path ending point position x, y and angle
  size_t p_size = previous_path_x.size();
  cout<<"p_size "<< p_size<<"\n";
  if(p_size == 0)
  {
      PrevPath.x_end = EgoVeh_info.x;
      PrevPath.y_end  = EgoVeh_info.y;
      PrevPath.yaw_end = EgoVeh_info.yaw;
  }
  else
  {
      PrevPath.x_end  = PrevPath.x[p_size-1];
      PrevPath.y_end  = PrevPath.y[p_size-1];

      double pos_x2 = PrevPath.x[p_size-2];
      double pos_y2 = PrevPath.y[p_size-2];
      PrevPath.yaw_end = atan2(PrevPath.y_end-pos_y2, PrevPath.x_end-pos_x2);
  }
}


void PP::update_env(vector<vector<double>> other_vehicles) {
    this->other_vehicles = other_vehicles;

}


void PP::FindFront(){
  int indx=0;
  double dis2frnt=10000;
  int N_veh = other_vehicles.size();
  for (int i =0; i< N_veh; i++){
     double s_diff = other_vehicles[i][5] - EgoVeh_info.s;
     double d_diff = other_vehicles[i][6] - EgoVeh_info.d;
     if (abs(d_diff)<2 && s_diff >=0 && s_diff < dis2frnt){
        indx = i;
        dis2frnt = other_vehicles[i][5];
     }
  }

  FrontVeh_info.x = other_vehicles[indx][1];
  FrontVeh_info.y = other_vehicles[indx][2];
  FrontVeh_info.vx= other_vehicles[indx][3];
  FrontVeh_info.vy = other_vehicles[indx][4];
  FrontVeh_info.s = other_vehicles[indx][5];
  FrontVeh_info.d = other_vehicles[indx][6];
  FrontVeh_info.speed = sqrt(FrontVeh_info.vx*FrontVeh_info.vx + FrontVeh_info.vy*FrontVeh_info.vy);
  //return {indx, dis2frnt};
}

void PP::ResetNewPath(){
   NewPath.x = {};
   NewPath.y = {};
   NewPath.s = {};
   NewPath.d = {};
   NewPath.yaw = {};
}


void PP::RetainPathXY() {
  ResetNewPath();
  for(int i = 0; i < PrevPath.path_size; i++)
  {   NewPath.x.push_back(PrevPath.x[i]);
      NewPath.y.push_back(PrevPath.y[i]);
      //NewPath.x[i]=  PrevPath.x[i];
      //NewPath.y[i] = PrevPath.y[i];
  }
}

void PP::generate_path_straight()
{
  ResetNewPath();
  double dist_inc = 0.5;
  for(int i = 0; i < path_size; i++)
  {
      NewPath.x.push_back(EgoVeh_info.x +(dist_inc*i)*cos(EgoVeh_info.yaw));
      NewPath.y.push_back(EgoVeh_info.y+(dist_inc*i)*sin(EgoVeh_info.yaw));
  }
}

void PP::generate_path_circle() {
  RetainPathXY();
  double dist_inc = 0.5;
  double pos_x = PrevPath.x_end;
  double pos_y = PrevPath.y_end;
  double angle = PrevPath.yaw_end;
  cout<<"angle" << angle << endl;
  size_t j=0;
  for(int i = PrevPath.path_size; i < path_size; i++)
  {
      /*NewPath.x[i] = pos_x+(dist_inc)*cos(angle+(j+1)*(M_PI/100.0));
      NewPath.y[i] = pos_y+(dist_inc)*cos(angle+(j+1)*(M_PI/100.0));

      NewPath.x.push_back(pos_x+dist_inc*cos(angle+(j+1)*(M_PI/100.0)));
      NewPath.y.push_back(pos_y+dist_inc*sin(angle+(j+1)*(M_PI/100.0)));
      pos_x = NewPath.x[i];
      pos_y = NewPath.y[i];*/
      NewPath.x.push_back(pos_x+(dist_inc)*cos(angle+(j+1)*M_PI/100));
      NewPath.y.push_back(pos_y+(dist_inc)*sin(angle+(j+1)*M_PI/100));
      pos_x += dist_inc*cos(angle+(j+1)*M_PI/100);
      pos_y += dist_inc*sin(angle+(j+1)*M_PI/100);

      j += 1;
  }
}


/*vector<double> PP::generate_path_MPC()
{
   FindFront();

   MPC mpc;
   Eigen::VectorXd coeffs(2);
   Eigen::VectorXd state(3);


   coeffs << FrontVeh_info.s, FrontVeh_info.speed;

   double s = EgoVeh_info.s;
   double speed = EgoVeh_info.speed;
   double cte = FrontVeh_info.s - EgoVeh_info.s;

   state <<s, speed, cte;
   vector<double> s_path = mpc.Solve(state, coeffs);

   return s_path;
}*/

void PP::update_path(vector<vector<double>> xy_path){
  RetainPathXY();
  for(int i=PrevPath.path_size; i < NewPath.path_size; i++)
  {
    NewPath.x.push_back(xy_path[0][i]);
    NewPath.y.push_back(xy_path[1][i]);
  }
}
