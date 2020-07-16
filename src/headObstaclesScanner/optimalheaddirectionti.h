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


const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;



class optimalHeadDirectionTi
{
public:
    optimalHeadDirectionTi();

    // VARIABLES

    // to be set:
    yarp::sig::Matrix robot_pose;
    yarp::sig::Matrix abs_corners;     //x, y, corner weight
    yarp::sig::Matrix abs_objects;     //x, y, object weight
    yarp::sig::Matrix abs_wayoints;    //x, y, waypoint weight

    double camera_fov = 70;
    double max_head_rotation = 25;
    double max_head_speed = 1000;
    double max_considered_radius = 3.5;


    // output
    double objective_value = -1;
    vector<double> X;

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
