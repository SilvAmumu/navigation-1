#include "optimalheaddirectionti.h"

optimalHeadDirectionTi::optimalHeadDirectionTi()
{

}

void optimalHeadDirectionTi::solveProblem()
{
    // get data

    //obtain polar coordinates
    obtainPolarCoordinates();

    // clean non relevant points
    cleanNonRelevantPoints();

    //initialize problem
    mip = glp_create_prob();
    glp_set_prob_name(mip, "headDirection");
    glp_set_obj_dir(mip, GLP_MAX);

    //set contraints

    //set boundaries

    //set objective function



}

void optimalHeadDirectionTi::obtainPolarCoordinates()
{
    // calculate polar coordinates in the ROBOT reference system
    polar_corners = abs_corners;
    polar_wayoints = abs_wayoints;
    double angle_temp;
    double radius_temp;

    // polar corners
    for (int i=0; i++; i<polar_corners.rows())
    {
        polar_corners(i,0) = abs_corners(i,0) - robot_pose(0,0);
        polar_corners(i,1) = abs_corners(i,1) - robot_pose(0,1);
        radius_temp = sqrt( pow(polar_corners(i,0),2) + pow(polar_corners(i,1),2) );
        angle_temp = atan2(polar_corners(i,1) , polar_corners(i,0));
        angle_temp = angle_temp * RAD2DEG;
        if (angle_temp < 0)
        {
            angle_temp = angle_temp + 360;
        }
        angle_temp = angle_temp - robot_pose(0,2);
        if (angle_temp > 180 )
            angle_temp = angle_temp - 360;

        polar_corners(i,0) = radius_temp;
        polar_corners(i,1) = angle_temp;
    }

    // polar waypoints
    for (int i=0; i++; i<polar_wayoints.rows())
    {
        polar_wayoints(i,0) = abs_wayoints(i,0) - robot_pose(0,0);
        polar_wayoints(i,1) = abs_wayoints(i,1) - robot_pose(0,1);
        radius_temp = sqrt( pow(polar_wayoints(i,0),2) + pow(polar_wayoints(i,1),2) );
        angle_temp = atan2(polar_wayoints(i,1) , polar_wayoints(i,0));
        angle_temp = angle_temp * RAD2DEG;
        if (angle_temp < 0)
        {
            angle_temp = angle_temp + 360;
        }
        angle_temp = angle_temp - robot_pose(0,2);
        if (angle_temp > 180 )
            angle_temp = angle_temp - 360;

        polar_wayoints(i,0) = radius_temp;
        polar_wayoints(i,1) = angle_temp;

    }


}

void optimalHeadDirectionTi::cleanNonRelevantPoints()
{
    // this function removes points out of reach from the RGBD fov or too far from the robot (to make MILP lighter)
    for (int i=0; i++; i<polar_corners.rows())
    {
        if (polar_corners(i,0) > max_considered_radius)
            polar_corners.removeRows(i,1);
    }

    for (int i=0; i++; i<polar_corners.rows())
    {
        if ((polar_corners(i,1) > max_head_rotation + camera_fov/2) ||  (polar_corners(i,1) < -max_head_rotation - camera_fov/2))
            polar_corners.removeRows(i,1);
    }

    for (int i=0; i++; i<polar_wayoints.rows())
    {
        if (polar_wayoints(i,0) > max_considered_radius)
            polar_wayoints.removeRows(i,1);
    }

    for (int i=0; i++; i<polar_wayoints.rows())
    {
        if ((polar_wayoints(i,1) > max_head_rotation + camera_fov/2) ||  (polar_wayoints(i,1) < -max_head_rotation - camera_fov/2))
            polar_wayoints.removeRows(i,1);
    }
}
