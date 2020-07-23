#include "optimalheaddirectiontv.h"

#define DEBUG

using namespace std::chrono;

//UTILITY FUNCTIONS
double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
   int size = xData.size();

   int i = 0;                                                                  // find left end of interval for interpolation
   if ( x >= xData[size - 2] )                                                 // special case: beyond right end
   {
      i = size - 2;
   }
   else
   {
      while ( x > xData[i+1] ) i++;
   }
   double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
   if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
   {
      if ( x < xL ) yR = yL;
      if ( x > xR ) yL = yR;
   }

   double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

   return yL + dydx * ( x - xL );                                              // linear interpolation
}



// MEMBERS

optimalHeadDirectionTv::optimalHeadDirectionTv()
{

}

void optimalHeadDirectionTv::solveProblem()  // TO DO: SET CONSTRAINTS ON MAX HEAD SPEED
{
    // get data

    // set time step and numbers of time step according to available trajectory
    if(abs_trajectory.rows() < 1)
    {
        trajectory_time = 0;
        number_time_steps = 1;
    }
    else
    {
        trajectory_time = abs_trajectory(abs_trajectory.rows()-1,6) - abs_trajectory(0,6);
        while((trajectory_time < number_time_steps * time_step) && (number_time_steps > 1))
        {
            number_time_steps = number_time_steps - 1;
        }
        // normalize time to 0
        double initial_time = abs_trajectory(0,6);
        for(int i=0; i<abs_trajectory.rows(); i++)
        {
            abs_trajectory(i,6) = abs_trajectory(i,6) - initial_time;
        }
    }

#ifdef DEBUG
    cout << " trajectory_time: " << trajectory_time << '\n';
    cout << " number_time_steps: " << number_time_steps << '\n';
#endif





    // set some variables

    double M_big = camera_fov + 360;
    double M_big2 = max_head_rotation * 3;

    // obtain polar coordinates of all the points (also in future)
    futurePointsCalculation();

    // clean non relevant points
    cleanNonRelevantPoints();

    // initialize problem
    mip = glp_create_prob();
    glp_set_prob_name(mip, "headDirection");
    glp_set_obj_dir(mip, GLP_MAX);

    // set contraints (rows)
    std::string constr_name;
    int constr_num =0;
    int constr_point_int_num = 0;

    constr_point_int_num = pol_points.rows() * 2;
    constr_num = constr_point_int_num + 4*number_time_steps +2*number_time_steps; //4* for abs(H) in the objective function and 2* for rotation speed limit
    glp_add_rows(mip, constr_num);

#ifdef DEBUG
    cout << "MILP OPTIMIZATION: " << '\n';
    cout << "abs_corners.rows(): " << abs_corners.rows() << '\n';
    cout << "abs_objects.rows(): " << abs_objects.rows() << '\n';
    cout << "abs_wayoints.rows(): " << abs_waypoints.rows() << '\n';
    cout << "pol_points.rows(): " << pol_points.rows() << '\n';
   // cout << pol_points.toString() << '\n';   //PRINT ALL THE POINTS, very long output
#endif

    for(int i=0; i<pol_points.rows(); i++)
    {
        // cout << "pol_points index: " << i << '\n';
        constr_name = "cUP" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+1, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+1, GLP_UP, 0.0, pol_points(i,1) + camera_fov/2 + M_big);

        constr_name = "cLO" + std::to_string(i+1);
        glp_set_row_name(mip, 2*i+2, constr_name.c_str());
        glp_set_row_bnds(mip, 2*i+2, GLP_LO, pol_points(i,1) - camera_fov/2 - M_big, 0.0);
    }

    // adjoint constraints for abs(Ht) in the objective function

    for(int i=0; i<number_time_steps; i++)
    {
        //adjt_1
        constr_name = "adj" + std::to_string(i+1) + "_1";
        glp_set_row_name(mip, constr_point_int_num + 1 + 6*i, constr_name.c_str());
        glp_set_row_bnds(mip, constr_point_int_num + 1 + 6*i, GLP_LO, 0.0, 0.0);

        //adjt_2
        constr_name = "adj" + std::to_string(i+1) + "_2";
        glp_set_row_name(mip, constr_point_int_num + 2 + 6*i, constr_name.c_str());
        glp_set_row_bnds(mip, constr_point_int_num + 2 + 6*i, GLP_LO, -M_big2, 0.0);

        //adjt_3
        constr_name = "adj" + std::to_string(i+1) + "_3";
        glp_set_row_name(mip, constr_point_int_num + 3 + 6*i, constr_name.c_str());
        glp_set_row_bnds(mip, constr_point_int_num + 3 + 6*i, GLP_UP, 0.0, 0.0);

        //adjt_4
        constr_name = "adj" + std::to_string(i+1) + "_4";
        glp_set_row_name(mip, constr_point_int_num + 4 + 6*i, constr_name.c_str());
        glp_set_row_bnds(mip, constr_point_int_num + 4 + 6*i, GLP_UP, 0.0, 0.0);

        if(i==0)  //if first point set initial head position
        {
            //speedt_1
            constr_name = "speed" + std::to_string(i+1) + "_1";
            glp_set_row_name(mip, constr_point_int_num + 5 + 6*i, constr_name.c_str());
            glp_set_row_bnds(mip, constr_point_int_num + 5 + 6*i, GLP_LO, -max_head_speed*time_step+initial_head_position, 0.0);

            //speedt_2
            constr_name = "speed" + std::to_string(i+1) + "_2";
            glp_set_row_name(mip, constr_point_int_num + 6 + 6*i, constr_name.c_str());
            glp_set_row_bnds(mip, constr_point_int_num + 6 + 6*i, GLP_UP, 0.0, max_head_speed*time_step+initial_head_position);
        }
        else
        {
            //speedt_1
            constr_name = "speed" + std::to_string(i+1) + "_1";
            glp_set_row_name(mip, constr_point_int_num + 5 + 6*i, constr_name.c_str());
            glp_set_row_bnds(mip, constr_point_int_num + 5 + 6*i, GLP_LO, -max_head_speed*time_step, 0.0);

            //speedt_2
            constr_name = "speed" + std::to_string(i+1) + "_2";
            glp_set_row_name(mip, constr_point_int_num + 6 + 6*i, constr_name.c_str());
            glp_set_row_bnds(mip, constr_point_int_num + 6 + 6*i, GLP_UP, 0.0, max_head_speed*time_step);
        }
    }



    //set states and boudaries
    std::string states_name;
    int states_num = 0;
    int states_point_int_num = 0;

    states_point_int_num = pol_points.rows();
    states_num = states_point_int_num + 3*number_time_steps;

    glp_add_cols(mip, states_num);

    // binary variables (referred to constraints)
    for(int i=0; i<states_point_int_num; i++)
    {
        if (pol_points(i,2) == 1)
            states_name = "Zc_" + std::to_string(i) + "_" + std::to_string(pol_points(i,4));
        if (pol_points(i,2) == 2)
            states_name = "Zo_" + std::to_string(i) + "_" + std::to_string(pol_points(i,4));
        if (pol_points(i,2) == 3)
            states_name = "Zw_" + std::to_string(i) + "_" + std::to_string(pol_points(i,4));

        glp_set_col_name(mip, i+1, states_name.c_str());
        glp_set_col_bnds(mip, i+1, GLP_DB, 0, 1);
        glp_set_col_kind(mip, i+1, GLP_IV);
    }

    // adjoint states for abs(Ht) in the objective function
    for(int i=0; i<number_time_steps; i++)
    {
        states_name = "H" + std::to_string(i+1);
        glp_set_col_name(mip, states_point_int_num + 1 + 3*i, states_name.c_str());
        glp_set_col_bnds(mip, states_point_int_num + 1 + 3*i, GLP_DB, -max_head_rotation, max_head_rotation);

        states_name = "Hadj" + std::to_string(i+1);
        glp_set_col_name(mip, states_point_int_num + 2 + 3*i, states_name.c_str());
        glp_set_col_bnds(mip, states_point_int_num + 2 + 3*i, GLP_UP, 0.0, max_head_rotation*2);

        states_name = "Hi" + std::to_string(i+1);
        glp_set_col_name(mip, states_point_int_num + 3 + 3*i, states_name.c_str());
        glp_set_col_bnds(mip, states_point_int_num + 3 + 3*i, GLP_DB, 0, 1);
        glp_set_col_kind(mip, states_point_int_num + 3 + 3*i, GLP_IV);
    }

    //set problem matrix
    int matrix_elements = constr_point_int_num * 2 + 10*number_time_steps +4*number_time_steps - 2;  //2 each constraint + 10 for adjoint per each time step + 4 for speed limit per each time step -2 for initial Ht-1 step for speed
    int cont =0;
    int ia[1+matrix_elements];
    int ja[1+matrix_elements];
    double ar[1+matrix_elements];
    int time_index;


    for(int i=0; i<pol_points.rows(); i++)
    {
        time_index = pol_points(i,4);
        // Ht index cUPi
        cont++;
        ia[cont] = 2*i + 1;
        ja[cont] = states_point_int_num + 1 + time_index*3;
        ar[cont] = 1;

        // Ht index cLOi
        cont++;
        ia[cont] = 2*i + 2;
        ja[cont] = states_point_int_num + 1 + time_index*3;;
        ar[cont] = 1;

        // Xi index cUPi
        cont++;
        ia[cont] = 2*i + 1;
        ja[cont] = (1 + i);
        ar[cont] = +M_big;

        // Xi index cLOi
        cont++;
        ia[cont] = 2*i + 2;
        ja[cont] = (1 + i);
        ar[cont] = -M_big;
    }

    for(int i=0; i<number_time_steps; i++)
    {
        // adjoint elements for abs(Ht) in the objective function
        //adjt_1 Ht
        cont++;
        ia[cont] = constr_point_int_num + 1 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = 1;
        //adjt_1 Hadjt
        cont++;
        ia[cont] = constr_point_int_num + 1 + 6*i;
        ja[cont] = states_point_int_num + 2 + 3*i;
        ar[cont] = -1;
        //adjt_1 Hit
        cont++;
        ia[cont] = constr_point_int_num + 1 + 6*i;
        ja[cont] = states_point_int_num + 3 + 3*i;
        ar[cont] = M_big2;

        //adjt_2 Ht
        cont++;
        ia[cont] = constr_point_int_num + 2 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = -1;
        //adjt_2 Hadjt
        cont++;
        ia[cont] = constr_point_int_num + 2 + 6*i;
        ja[cont] = states_point_int_num + 2 + 3*i;
        ar[cont] = -1;
        //adjt_2 Hit
        cont++;
        ia[cont] = constr_point_int_num + 2 + 6*i;
        ja[cont] = states_point_int_num + 3 + 3*i;
        ar[cont] = -M_big2;

        //adjt_3 Ht
        cont++;
        ia[cont] = constr_point_int_num + 3 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = 1;
        //adjt_3 Hadjt
        cont++;
        ia[cont] = constr_point_int_num + 3 + 6*i;
        ja[cont] = states_point_int_num + 2 + 3*i;
        ar[cont] = -1;

        //adjt_4 Ht
        cont++;
        ia[cont] = constr_point_int_num + 4 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = -1;

        //adjt_4 Hadjt
        cont++;
        ia[cont] = constr_point_int_num + 4 + 6*i;
        ja[cont] = states_point_int_num + 2 + 3*i;
        ar[cont] = -1;


        //speedt_1 Ht
        cont++;
        ia[cont] = constr_point_int_num + 5 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = 1;

        //speedt_1 Ht-1
        if (i==0)
        {
            //cont++;
            //ia[cont] = constr_point_int_num + 5 + 6*i;
            //ja[cont] = states_point_int_num + 1 + 3*i;
            //ar[cont] = -1;
        }
        else
        {
            cont++;
            ia[cont] = constr_point_int_num + 5 + 6*i;
            ja[cont] = states_point_int_num + 1 + 3*(i-1);
            ar[cont] = -1;
        }

        //speedt_2 Ht
        cont++;
        ia[cont] = constr_point_int_num + 6 + 6*i;
        ja[cont] = states_point_int_num + 1 + 3*i;
        ar[cont] = 1;

        //speedt_2 Ht-1
        if (i==0)
        {
            //cont++;
            //ia[cont] = constr_point_int_num + 6 + 6*i;
            //ja[cont] = states_point_int_num + 1 + 3*i;
            //ar[cont] = -1;
        }
        else
        {
            cont++;
            ia[cont] = constr_point_int_num + 6 + 6*i;
            ja[cont] = states_point_int_num + 1 + 3*(i-1);
            ar[cont] = -1;
        }
    }

    glp_load_matrix(mip, matrix_elements, ia, ja, ar);


    //set objective function

    // integer states
    const char *type_name;
    double weight_coeff = 0;
    double point_distance = 0;

    for(int i=0; i<states_point_int_num; i++)
    {
        type_name = glp_get_col_name(mip,i+1);
        if (type_name[1] == 'c')
            weight_coeff = std_weight_corners;
        if (type_name[1] == 'o')
            weight_coeff = std_weight_objects;
        if (type_name[1] == 'w')
            weight_coeff = std_weight_waypoints;

        point_distance = pol_points(i,0);
        if (point_distance>1)
            weight_coeff = weight_coeff / (point_distance * 2);

        glp_set_obj_coef(mip, i+1, weight_coeff);

    }
    //the Hadjt
    for(int i=0; i<number_time_steps; i++)
    {
        glp_set_obj_coef(mip, states_point_int_num + 2 +3*i, std_weight_absHeading);
    }

    //solve the problem
    glp_iocp parm;
    glp_init_iocp(&parm);
    parm.presolve = GLP_ON;
    parm.tm_lim = opti_time_limit;

    auto start = std::chrono::high_resolution_clock::now();
    int err = glp_intopt(mip, &parm);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<milliseconds>(stop - start);
    cout << "MILP duration (milliseconds): " << duration.count() << '\n';

    //return results
    objective_value = glp_mip_obj_val(mip);
    solutions.resize(states_num);
    optimal_head_direction = glp_mip_col_val(mip, states_point_int_num+1);
    for(int i=0; i<states_num; i++)
    {
        solutions[i] = glp_mip_col_val(mip, i+1);
    }

    //print problem statement
#ifdef DEBUG
    glp_write_lp(mip, NULL, "problem_statement_outputTV.txt");
    glp_print_mip(mip,"Problem_solutionTV.txt");
    cout << "objective_value: " << objective_value << '\n';
    cout << "MILP solution: " << '\n';
    for(int i=0; i<states_num; i++)
    {
        // cout << "X" << i << ": " << solutions[i] << '\n'; //PRINT ALL THE POINTS, very long output
    }
    for(int i=0; i<number_time_steps; i++)
    {
         //cout << "H" << i << ": " << solutions[states_point_int_num + 1 + 3*i] << '\n';
         cout << glp_get_col_name(mip,states_point_int_num + 1 + 3*i) << ": " << solutions[states_point_int_num + 1 + 3*i] << '\n';
    }
    cout << "Initial head position: " << initial_head_position << '\n';

#endif
}

yarp::sig::Matrix optimalHeadDirectionTv::obtainPolarCoordinates(yarp::sig::Matrix robot_posef)
{
    // calculate polar coordinates in the ROBOT reference system
    abs_points.resize(abs_corners.rows() + abs_waypoints.rows() + abs_objects.rows(), 4);

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
    for (int i=0; i<abs_waypoints.rows(); i++)
    {
        abs_points(cont,0) = abs_waypoints(i,0);
        abs_points(cont,1) = abs_waypoints(i,1);
        abs_points(cont,2) = 3;
        abs_points(cont,3) = -1;
        cont ++;
    }

    yarp::sig::Matrix pol_pointsf;
    pol_pointsf = abs_points;

    double angle_temp;
    double radius_temp;

    // polar corners
    for (int i=0; i<pol_pointsf.rows(); i++)
    {
        pol_pointsf(i,0) = abs_points(i,0) - robot_posef(0,0);
        pol_pointsf(i,1) = abs_points(i,1) - robot_posef(0,1);
        radius_temp = sqrt( pow(pol_pointsf(i,0),2) + pow(pol_pointsf(i,1),2) );
        angle_temp = atan2(pol_pointsf(i,1) , pol_pointsf(i,0));
        angle_temp = angle_temp * RAD2DEG;
        if (angle_temp < 0)
        {
            angle_temp = angle_temp + 360;
        }
        angle_temp = angle_temp - robot_posef(0,2);
        if (angle_temp > 180 )
            angle_temp = angle_temp - 360;

        pol_pointsf(i,0) = radius_temp;
        pol_pointsf(i,1) = angle_temp;
    }
    return pol_pointsf;
}

void optimalHeadDirectionTv::cleanNonRelevantPoints()
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

void optimalHeadDirectionTv::futurePointsCalculation()
{
    yarp::sig::Matrix robot_pose_t;
    yarp::sig::Matrix pol_points_t;
    pol_points.resize((abs_corners.rows() + abs_waypoints.rows() + abs_objects.rows())*number_time_steps, 5);

    int cont = 0;
    for(int i=0; i<number_time_steps; i++)
    {
        //obtain absolute robot position at time t
        robot_pose_t = futureRobotPosition(robot_pose,abs_trajectory,i*time_step);

        //obtain polar coordinates at time t
        pol_points_t = obtainPolarCoordinates(robot_pose_t);

#ifdef DEBUG
        cout << "ROBOT POSE AT TIME: " << i*time_step << " is: \n" << robot_pose_t.toString() << "\n interpolated from: \n " << abs_trajectory.toString() << '\n';
#endif

        //create a single matrix
        for(int j=0; j<pol_points_t.rows(); j++)
        {
            pol_points(cont,0) = pol_points_t(j,0);     // x
            pol_points(cont,1) = pol_points_t(j,1);     // y
            pol_points(cont,2) = pol_points_t(j,2);     // type
            pol_points(cont,3) = pol_points_t(j,3);     // weight
            pol_points(cont,4) = i;                     // time index
            cont++;
        }
    }
}

yarp::sig::Matrix optimalHeadDirectionTv::futureRobotPosition(yarp::sig::Matrix m_robot_pose, yarp::sig::Matrix m_abs_trajectory, double time)
{
    yarp::sig::Matrix future_robot_position = m_robot_pose;
    if(m_abs_trajectory.rows()>1)
    {
        if(time > 0)
        {
            vector<double> xData(m_abs_trajectory.rows(),0);
            vector<double> yData0(m_abs_trajectory.rows(),0);
            vector<double> yData1(m_abs_trajectory.rows(),0);
            vector<double> yData2(m_abs_trajectory.rows(),0);

            for(int i=0; i<m_abs_trajectory.rows(); i++ )
            {
                xData[i] = m_abs_trajectory(i,6);
                yData0[i] = m_abs_trajectory(i,0);
                yData1[i] = m_abs_trajectory(i,1);
                yData2[i] = m_abs_trajectory(i,2);
            }
            future_robot_position(0,0) = interpolate(xData, yData0, time, true);
            future_robot_position(0,1) = interpolate(xData, yData1, time, true);
            future_robot_position(0,2) = interpolate(xData, yData2, time, true);
        }
    }
    return future_robot_position;
}


