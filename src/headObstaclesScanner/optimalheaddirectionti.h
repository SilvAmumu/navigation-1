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

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

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
    yarp::sig::Matrix abs_wayoints;    //x, y, waypoint weight
    yarp::sig::Matrix polar_corners;   //r, t
    yarp::sig::Matrix polar_wayoints;   //r, t

    double camera_fov;
    double max_head_rotation;
    double max_considered_radius;


    // output

    //service
    glp_prob *mip;




    // METHODS
    void solveProblem(void);


private:

    // VARIABLES


    // METHODS
    void obtainPolarCoordinates(void);

    void cleanNonRelevantPoints(void);

};

#endif // OPTIMALHEADDIRECTIONTI_H
