#include "optimalheaddirectionti.h"

#define DEBUG

using namespace std::chrono;

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

    constr_num = pol_points.rows()*2;
    glp_add_rows(mip, constr_num);

#ifdef DEBUG
    cout << "MILP OPTIMIZATION: " << '\n';
    cout << "abs_corners.rows(): " << abs_corners.rows() << '\n';
    cout << "abs_objects.rows(): " << abs_objects.rows() << '\n';
    cout << "abs_wayoints.rows(): " << abs_wayoints.rows() << '\n';
    cout << "pol_points.rows(): " << pol_points.rows() << '\n';
    cout << pol_points.toString() << '\n';
#endif

    for (int i=0; i<pol_points.rows(); i++)
    {
        //cout << "pol_points index: " << i << '\n';
        constr_name = "cUP" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+1, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+1, GLP_UP, 0.0, pol_points(i,1) + camera_fov/2 + M_big);

        constr_name = "cLO" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+2, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+2, GLP_LO, pol_points(i,1) - camera_fov/2 - M_big, 0.0);
    }

    //set states and boudaries
    std::string states_name;
    int states_num = 0;

    states_num = 1 + pol_points.rows();
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


    for(int i=0; i<pol_points.rows(); i++)
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
        ar[cont] = +M_big;

        // Xi index cLOi
        cont++;
        ia[cont] = 2*i + 2;
        ja[cont] = (1 + i) + 1;
        ar[cont] = -M_big;
    }

    glp_load_matrix(mip, matrix_elements, ia, ja, ar);


    //set objective function
    for(int i=0; i<states_num; i++)
    {
        glp_set_obj_coef(mip, i+1, 1.0);
    }

    //solve the problem
    glp_iocp parm;
    glp_init_iocp(&parm);
    parm.presolve = GLP_ON;

    auto start = std::chrono::high_resolution_clock::now();
    int err = glp_intopt(mip, &parm);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<milliseconds>(stop - start);
    cout << "MILP duration (milliseconds): " << duration.count() << '\n';

    //return results
    objective_value = glp_mip_obj_val(mip);
    X.resize(states_num);
    for(int i=0; i<states_num; i++)
    {
        X[i] = glp_mip_col_val(mip, i+1);
    }

    //print problem statement
#ifdef DEBUG
    glp_write_lp(mip, NULL, "problem_statement_output.txt");
    glp_print_mip(mip,"Problem_solution.txt");
    cout << "objective_value: " << objective_value << '\n';
    cout << "MILP solution: " << '\n';
    for(int i=0; i<states_num; i++)
    {
        cout << "X" << i << ": " << X[i] << '\n';
    }
#endif
}

void optimalHeadDirectionTi::obtainPolarCoordinates()
{
    // calculate polar coordinates in the ROBOT reference system  
    abs_points.resize(abs_corners.rows() + abs_wayoints.rows() + abs_objects.rows(), 4);

    int cont = 0;
    for (int i=0; i<abs_corners.rows(); i++)
    {
        abs_points(cont,0) = abs_corners(i,0);
        abs_points(cont,1) = abs_corners(i,1);
        abs_points(cont,2) = 1;
        abs_points(cont,3) = -1;
        cont ++;
    }
    for (int i=0; i<abs_objects.rows(); i++)
    {
        abs_points(cont,0) = abs_objects(i,0);
        abs_points(cont,1) = abs_objects(i,1);
        abs_points(cont,2) = 2;
        abs_points(cont,3) = -1;
        cont ++;
    }
    for (int i=0; i<abs_wayoints.rows(); i++)
    {
        abs_points(cont,0) = abs_wayoints(i,0);
        abs_points(cont,1) = abs_wayoints(i,1);
        abs_points(cont,2) = 3;
        abs_points(cont,3) = -1;
        cont ++;
    }

    pol_points = abs_points;

    double angle_temp;
    double radius_temp;

    // polar corners
    for (int i=0; i<pol_points.rows(); i++)
    {
        pol_points(i,0) = abs_points(i,0) - robot_pose(0,0);
        pol_points(i,1) = abs_points(i,1) - robot_pose(0,1);
        radius_temp = sqrt( pow(pol_points(i,0),2) + pow(pol_points(i,1),2) );
        angle_temp = atan2(pol_points(i,1) , pol_points(i,0));
        angle_temp = angle_temp * RAD2DEG;
        if (angle_temp < 0)
        {
            angle_temp = angle_temp + 360;
        }
        angle_temp = angle_temp - robot_pose(0,2);
        if (angle_temp > 180 )
            angle_temp = angle_temp - 360;

        pol_points(i,0) = radius_temp;
        pol_points(i,1) = angle_temp;
    }
}

void optimalHeadDirectionTi::cleanNonRelevantPoints()
{
    // this function removes points out of reach from the RGBD fov or too far from the robot (to make MILP lighter)
    for (int i=0; i<pol_points.rows(); i++)
    {
        if (pol_points(i,0) > max_considered_radius)
            pol_points.removeRows(i,1);
    }

    for (int i=0; i<pol_points.rows(); i++)
    {
        if ((pol_points(i,1) > max_head_rotation + camera_fov/2) ||  (pol_points(i,1) < -max_head_rotation - camera_fov/2))
            pol_points.removeRows(i,1);
    }
}
