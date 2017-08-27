#ifndef MAP_H
#define MAP_H

#include <fstream>
#include <iostream>
#include <vector>
#include "helpfunc.h"


class Map {

  public:

    Points x; // map coordinates
    Points y;
    Points s; // distance from starting point
    // vector orthogonal to road
    Points dx;
    Points dy;

    const double LANE_WIDTH = 4; /*meters*/
    const int RIGHTMOST_LANE = 2; /*lane 0, 1, 2*/
    const double MAX_S = 6945.554; // road maximum length

    // mapping from (s) -> (x, y) for each lane using spline

    vector<Trajectory> lane_s2x;
    vector<Trajectory> lane_s2y;

public: // functions
 // Map constrcutor function
    Map(const Points & x,const Points & y,const Points & s,
        const Points & dx, const Points & dy);

    virtual ~Map();
// member functions
    int find_lane(double car_d) const;// return lane index (0,1,2)
    int find_front_car_in_lane(const EgoCarInfo & ego,
                               const vector<PeerCar> & peer_cars,int lane) const;
    int find_rear_car_in_lane(const EgoCarInfo & ego,
                              const vector<PeerCar> & peer_cars,
                              int lane) const;
};

#endif
