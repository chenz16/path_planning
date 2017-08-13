Eigen::VectorXd coeffs(2);
Eigen::VectorXd state(3);
coeffs << pp.FrontVeh_info.s, pp.FrontVeh_info.speed;

double s = pp.EgoVeh_info.s;
double speed = pp.EgoVeh_info.speed;
double cte = pp.FrontVeh_info.s - pp.EgoVeh_info.s;

state <<s, speed, cte;
//vector<double> mpc_path=mpc.Solve(state, coeffs);
//vector<double> start_path;
vector<double> s_path (50);
//vector<double> ss_path;

//if (veh_start==false) {
  for (int i=0; i< 50; i++)
  {
    s_path[i]=car_s +2.0*i*0.02;
  }
/*  if (car_speed > 1) {veh_start = true;}

} else { s_path = mpc.Solve(state, coeffs);}*/

  //if (veh_start==false && car_speed<0.5)
  //{ s_path = start_path;
    //veh_start = true;}
  //else {s_path = mpc_path;}

// MPC
    size_t N = 15;
    mpc.set_para(15);
    //cout<<mpc.v_start<<endl;
    vector<double> ss_path=mpc.Solve(state, coeffs);



cout<<"veh_start" << veh_start<<endl;
