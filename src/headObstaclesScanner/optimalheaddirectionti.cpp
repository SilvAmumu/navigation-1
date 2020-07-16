#include "optimalheaddirectionti.h"

optimalHeadDirectionTi::optimalHeadDirectionTi()
{

}

void optimalHeadDirectionTi::solveProblem()
{
    // get data
    
    
    // set some variables
    
    double M_big = camera_fov +360;

    //obtain polar coordinates
    obtainPolarCoordinates();

    // clean non relevant points
    cleanNonRelevantPoints();

    //initialize problem
    mip = glp_create_prob();
    glp_set_prob_name(mip, "headDirection");
    glp_set_obj_dir(mip, GLP_MAX);

    //set contraints (rows)
    std::string constr_name;
    int constr_num =0;

    constr_num = (polar_corners.rows() + polar_objects.rows() + polar_wayoints.rows())*2;
    glp_add_rows(mip, constr_num);

    cout << "polar_corners.rows(): " << polar_corners.rows() << '\n';
    cout << "polar_objects.rows(): " << polar_objects.rows() << '\n';
    cout << "polar_wayoints.rows(): " << polar_wayoints.rows() << '\n';
    cout << polar_corners.toString() << '\n';

    for (int i=0; i<polar_corners.rows(); i++)
    {
        //cout << "polar_corners index: " << i << '\n';
        constr_name = "cUP" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+1, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+1, GLP_UP, 0.0, polar_corners(i,1) + camera_fov/2);

        constr_name = "cLO" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+2, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+2, GLP_LO, polar_corners(i,1) - camera_fov/2, 0.0);
    }

    for (int i=polar_corners.rows(); i<(polar_corners.rows() + polar_objects.rows()); i++)
    {
        //cout << "polar_objects index: " << i << '\n';
        constr_name = "oUP" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+1, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+1, GLP_UP, 0.0, polar_objects(i,1) + camera_fov/2);

        constr_name = "oLO" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+2, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+2, GLP_LO, polar_objects(i,1) - camera_fov/2, 0.0);
    }

    for (int i=polar_corners.rows() + polar_objects.rows(); i<(polar_corners.rows() + polar_objects.rows() + polar_wayoints.rows()); i++)
    {
        //cout << "polar_waypoints index: " << i << '\n';
        constr_name = "wUP" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+1, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+1, GLP_UP, 0.0, polar_wayoints(i,1) + camera_fov/2);

        constr_name = "wLO" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+2, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+2, GLP_LO, polar_wayoints(i,1) - camera_fov/2, 0.0);
    }

    //set states and boudaries
    std::string states_name;
    int states_num = 0;

    states_num = 1 + (polar_corners.rows() + polar_objects.rows() + polar_wayoints.rows());
    glp_add_cols(mip, states_num);

    glp_set_col_name(mip, 1, "H");
    glp_set_col_bnds(mip, 1, GLP_DB, -max_head_rotation, max_head_rotation);

    for(int i=1; i<states_num; i++)
    {
        states_name = "z" + std::to_string(i);
        glp_set_col_name(mip, i+1, states_name.c_str());
        glp_set_col_bnds(mip, i+1, GLP_DB, 0, 1);
        glp_set_col_kind(mip, i+1, GLP_IV);
    }

    //set problem matrix
    int matrix_elements = constr_num * 2;
    int cont =0;
    int ia[1+matrix_elements];
    int ja[1+matrix_elements];
    double ar[1+matrix_elements];


    for(int i=0; i<(polar_corners.rows() + polar_objects.rows() + polar_wayoints.rows()); i++)
    {
        // H index cUPi
        cont++;
        ia[cont] = 2*i + 1;
        ja[cont] = 1;
        ar[cont] = 1;

        // H index cLOi
        cont++;
        ia[cont] = 2*i + 2;
        ja[cont] = 1;
        ar[cont] = 1;

        // Xi index cUPi
        cont++;
        ia[cont] = 2*i + 1;
        ja[cont] = (1 + i) + 1;
        ar[cont] = -M_big;

        // Xi index cLOi
        cont++;
        ia[cont] = 2*i + 2;
        ja[cont] = (1 + i) + 1;
        ar[cont] = M_big;
    }

    glp_load_matrix(mip, matrix_elements, ia, ja, ar);


    //set objective function
    for(int i=0; i<states_num; i++)
    {
        glp_set_obj_coef(mip, i+1, 1.0);
    }

    //print problem statement
    glp_iocp parm;
    glp_init_iocp(&parm);
    parm.presolve = GLP_ON;
    int err = glp_intopt(mip, &parm);

    glp_write_lp(mip, NULL, "test1");


}

void optimalHeadDirectionTi::obtainPolarCoordinates()
{
    // calculate polar coordinates in the ROBOT reference system
    polar_corners = abs_corners;
    //cout << "abs_corners assigned rows " << abs_corners.rows() << '\n';
    //cout << "polar_corners assigned rows " << polar_corners.rows() << '\n';
    polar_wayoints = abs_wayoints;
    polar_objects = abs_objects;
    double angle_temp;
    double radius_temp;

    // polar corners
    for (int i=0; i<polar_corners.rows(); i++)
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

    // polar objects
    for (int i=0; i<polar_objects.rows(); i++)
    {
        polar_objects(i,0) = abs_objects(i,0) - robot_pose(0,0);
        polar_objects(i,1) = abs_objects(i,1) - robot_pose(0,1);
        radius_temp = sqrt( pow(polar_objects(i,0),2) + pow(polar_objects(i,1),2) );
        angle_temp = atan2(polar_objects(i,1) , polar_objects(i,0));
        angle_temp = angle_temp * RAD2DEG;
        if (angle_temp < 0)
        {
            angle_temp = angle_temp + 360;
        }
        angle_temp = angle_temp - robot_pose(0,2);
        if (angle_temp > 180 )
            angle_temp = angle_temp - 360;

        polar_objects(i,0) = radius_temp;
        polar_objects(i,1) = angle_temp;
    }

    // polar waypoints
    for (int i=0; i<polar_wayoints.rows(); i++)
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
    for (int i=0; i<polar_corners.rows(); i++)
    {
        if (polar_corners(i,0) > max_considered_radius)
            polar_corners.removeRows(i,1);
    }

    for (int i=0; i<polar_corners.rows(); i++)
    {
        if ((polar_corners(i,1) > max_head_rotation + camera_fov/2) ||  (polar_corners(i,1) < -max_head_rotation - camera_fov/2))
            polar_corners.removeRows(i,1);
    }

    for (int i=0; i<polar_wayoints.rows(); i++)
    {
        if (polar_wayoints(i,0) > max_considered_radius)
            polar_wayoints.removeRows(i,1);
    }

    for (int i=0; i<polar_wayoints.rows(); i++)
    {
        if ((polar_wayoints(i,1) > max_head_rotation + camera_fov/2) ||  (polar_wayoints(i,1) < -max_head_rotation - camera_fov/2))
            polar_wayoints.removeRows(i,1);
    }
}
