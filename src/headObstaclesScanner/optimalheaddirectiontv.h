#ifndef OPTIMALHEADDIRECTIONTV_H
#define OPTIMALHEADDIRECTIONTV_H

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




class optimalHeadDirectionTv
{
public:
    optimalHeadDirectionTv();

    // VARIABLES

    // to be set:
    yarp::sig::Matrix robot_pose;
    yarp::sig::Matrix abs_corners;       //x, y, corner weight
    yarp::sig::Matrix abs_objects;       //x, y, object weight
    yarp::sig::Matrix abs_waypoints;      //x, y, waypoint weight
    yarp::sig::Matrix abs_trajectory;    //x, y
    double initial_head_position;

    double camera_fov = 70;
    double max_head_rotation = 35;
    double max_head_speed = 25;
    double max_considered_radius = 3.5;
    double std_weight_absHeading = 0.1;
    double std_weight_corners = 1;
    double std_weight_objects = 4;
    double std_weight_waypoints = 4;
    double time_step = 0.5;
    double number_time_steps = 7;
    int opti_time_limit = 400;



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
    yarp::sig::Matrix pol_points;  //r, t, type, weight, time step
    double trajectory_time;


    // METHODS
    yarp::sig::Matrix obtainPolarCoordinates(yarp::sig::Matrix robot_posef);

    void cleanNonRelevantPoints(void);

    void futurePointsCalculation(void);

    yarp::sig::Matrix futureRobotPosition(yarp::sig::Matrix m_robot_pose, yarp::sig::Matrix m_abs_trajectory, double time);

};

#endif // OPTIMALHEADDIRECTIONTV_H
