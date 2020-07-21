#ifndef OPTIMALHEADDIRECTIONTI_H
#define OPTIMALHEADDIRECTIONTI_H

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <glpk.h>
#include <chrono>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265
#endif


#ifndef RAD2DEG
#define RAD2DEG (180.0 / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0)
#endif



class optimalHeadDirectionTi
{
public:
    optimalHeadDirectionTi();

    // VARIABLES

    // to be set:
    yarp::sig::Matrix robot_pose;
    yarp::sig::Matrix abs_corners;     //x, y, corner weight
    yarp::sig::Matrix abs_objects;     //x, y, object weight
    yarp::sig::Matrix abs_waypoints;    //x, y, waypoint weight

    double camera_fov = 70;
    double max_head_rotation = 35;
    double max_head_speed = 1000;
    double max_considered_radius = 3.5;
    double std_weight_absHeading = 0.1;
    double std_weight_corners = 1;
    double std_weight_objects = 4;
    double std_weight_waypoints = 4;



    // output
    double objective_value = -1;
    double optimal_head_direction = 0;
    vector<double> solutions;

    //service
    glp_prob *mip;




    // METHODS
    void solveProblem(void);


private:

    // VARIABLES   
    yarp::sig::Matrix abs_points;  //r, t, type, weight (type: 1-corners , 2-objects, 3-waypoint)
    yarp::sig::Matrix pol_points;  //r, t, type, weight


    // METHODS
    void obtainPolarCoordinates(void);

    void cleanNonRelevantPoints(void);

};

#endif // OPTIMALHEADDIRECTIONTI_H
