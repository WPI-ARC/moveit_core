/*****partial jerk limit**************/


/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ken Anderson */

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/JointLimits.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>


//reflexxes includes
//#include <ReflexxesAPI.h>
//#include <RMLPositionFlags.h>
//#include <RMLPositionInputParameters.h>
//#include <RMLPositionOutputParameters.h>




#include "ros/ros.h"

//#define CYCLE_TIME_IN_SECONDS                   0.001
//#define NUMBER_OF_DOFS                          1

//int                         ResultValue                 =   0       ;

//ReflexxesAPI                *RML                        =   NULL    ;
  
//RMLPositionInputParameters  *IP                         =   NULL    ;
    
//RMLPositionOutputParameters *OP                         =   NULL    ;
    
//RMLPositionFlags            Flags                                   ;



namespace trajectory_processing
{

//already in map order
double jerkLimit[] = {4.5, 4.52, 4.5, 4.5, 4.5, 4.5, 4.5};
static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 1.0;
static const double ROUNDING_THRESHOLD = 0.01;
double time_diff_before[100];
double a_profile[100];
double a_profile_before[100];
double prev_a[7] = {0, 0, 0, 0, 0, 0, 0};
bool applyJerkLimit=false;
double first_a[7];
double last_a[7];

double velocity_limit_factor=1;



/*void applyRML(robot_trajectory::RobotTrajectory& rob_trajectory,
              const std::vector<std::string>& active_joints,
              const std::vector<moveit_msgs::JointLimits>& limits,
              std::vector<double> &time_diff) 
{
  RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                          ,   CYCLE_TIME_IN_SECONDS   );  
  IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );   
  OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );
  IP->SelectionVector->VecData            [0] =   true        ;

	bool startPoint_found = false;
	bool endPoint_found = false;
	bool direction_up = false;

	double startPoint_state;
	double endPoint_state;


  const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
  const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
  const unsigned int num_points = rob_trajectory.getWayPointCount();
  const unsigned int num_joints = group->getVariableCount();
  robot_state::RobotStatePtr curr_waypoint;
  robot_state::RobotStatePtr next_waypoint;
  std::map<std::string, double> curr_state_values;
  std::map<std::string, double> next_state_values;
	double dq1;
  double dq2;

	for(unsigned int joint=0; joint < active_joints.size(); joint++)
  {
		startPoint_found = false;
		endPoint_found = false;

    for ( unsigned int i=0; i<num_points; i++ )
    {
      double v_max = 1.0;
			
			if (i != num_points - 1)
			{
    		curr_waypoint = rob_trajectory.getWayPointPtr(i);
    		next_waypoint = rob_trajectory.getWayPointPtr(i+1);
    		curr_waypoint->getStateValues(curr_state_values);
    		next_waypoint->getStateValues(next_state_values);
      	dq1 = curr_state_values[active_joints[joint]];
      	dq2 = next_state_values[active_joints[joint]];
			}
			else 
			{
				curr_waypoint = rob_trajectory.getWayPointPtr(i);  		
    		curr_waypoint->getStateValues(curr_state_values);
				dq1 = curr_state_values[active_joints[joint]];
			}



			if (i == 0)
			{
				startPoint_found = true;
				startPoint_state = dq1;	
				if ( (dq2 - dq1) >= 0)
				{
					direction_up = true;
				}
				else
				{
					direction_up = false;
				}
			}
			else if (i == num_points - 1)
			{
				endPoint_found = true;
				endPoint_state = dq1;
			}
			else
			{
				if (direction_up == true)
				{
					if ((dq2 - dq1) < 0)
					{
						endPoint_found = true;
						endPoint_state = dq1;
					}
					else
					{
						endPoint_found = false;		
					}
				}
				else
				{
					if ((dq2 - dq1) >= 0)
					{
						endPoint_found = true;
						endPoint_state = dq1;
					}
					else
					{
						endPoint_found = false;		
					}
				}
				

			}
			
			if (endPoint_found == true)
			{
				printf("--------------------------------Joint %d-------------------------------------\n", joint);
				printf("\ni: %d, startPoint_state: %f , endPoint_state: %f \n", i, startPoint_state, endPoint_state);
				if (i != num_points - 1)
				{

					if ( (dq2 - dq1) >= 0)
					{
						direction_up = true;
					}
					else
					{
						direction_up = false;
					}
				}
				if (i != num_points - 1)
				{
					startPoint_state = endPoint_state;
				}
			}

    }
  }


	
}*/



IterativeParabolicTimeParameterization::IterativeParabolicTimeParameterization(unsigned int max_iterations,
                                                                               double max_time_change_per_it)
  : max_iterations_(max_iterations),
    max_time_change_per_it_(max_time_change_per_it)
{}

IterativeParabolicTimeParameterization::~IterativeParabolicTimeParameterization()
{}

void IterativeParabolicTimeParameterization::printPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const
{
  logDebug(  " time   [%i]= %f",i,point.time_from_start.toSec());
  if(point.positions.size() >= 7 )
  {
    logDebug(" pos_   [%i]= %f %f %f %f %f %f %f",i,
             point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
  }
  if(point.velocities.size() >= 7 )
  {
    logDebug("  vel_  [%i]= %f %f %f %f %f %f %f",i,
             point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
  }
  if(point.accelerations.size() >= 7 )
  {
    logDebug("   acc_ [%i]= %f %f %f %f %f %f %f",i,
             point.accelerations[0],point.accelerations[1],point.accelerations[2],point.accelerations[3],point.accelerations[4],point.accelerations[5],point.accelerations[6]);
  }
}

void IterativeParabolicTimeParameterization::printStats(const trajectory_msgs::JointTrajectory& trajectory,
                                                        const std::vector<moveit_msgs::JointLimits>& limits) const
{
  logDebug("jointNames= %s %s %s %s %s %s %s",
           limits[0].joint_name.c_str(),limits[1].joint_name.c_str(),limits[2].joint_name.c_str(),
           limits[3].joint_name.c_str(),limits[4].joint_name.c_str(),limits[5].joint_name.c_str(),
           limits[6].joint_name.c_str());
  logDebug("maxVelocities= %f %f %f %f %f %f %f",
           limits[0].max_velocity,limits[1].max_velocity,limits[2].max_velocity,
           limits[3].max_velocity,limits[4].max_velocity,limits[5].max_velocity,
           limits[6].max_velocity);
  logDebug("maxAccelerations= %f %f %f %f %f %f %f",
           limits[0].max_acceleration,limits[1].max_acceleration,limits[2].max_acceleration,
           limits[3].max_acceleration,limits[4].max_acceleration,limits[5].max_acceleration,
           limits[6].max_acceleration);
  // for every point in time:
  for (unsigned int i=0; i<trajectory.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint& point = trajectory.points[i];
    printPoint(point, i);
  }
}

// Applies velocity
void IterativeParabolicTimeParameterization::applyVelocityConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                                                      const std::vector<std::string>& active_joints,
                                                                      const std::vector<moveit_msgs::JointLimits>& limits,
                                                                      std::vector<double> &time_diff) const
{



  robot_state::RobotStatePtr curr_waypoint;
  robot_state::RobotStatePtr next_waypoint;
  std::map<std::string, double> curr_state_values;
  std::map<std::string, double> next_state_values;

  const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
  const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();

  const unsigned int num_points = rob_trajectory.getWayPointCount();
  const unsigned int num_joints = group->getVariableCount();

  for( unsigned int i=0; i<num_points-1; ++i )
  {
    curr_waypoint = rob_trajectory.getWayPointPtr(i);
    next_waypoint = rob_trajectory.getWayPointPtr(i+1);

    curr_waypoint->getStateValues(curr_state_values);
    next_waypoint->getStateValues(next_state_values);

    for (unsigned int joint=0; joint < active_joints.size(); joint++)
    {
      double v_max = 1.0;

      if( limits[joint].has_velocity_limits )
      {
        v_max = limits[joint].max_velocity * velocity_limit_factor;
      }
      const double dq1 = curr_state_values[active_joints[joint]];
      const double dq2 = next_state_values[active_joints[joint]];
      const double t_min = std::abs(dq2-dq1) / v_max;
      if( t_min > time_diff[i] )
      {
        time_diff[i] = t_min;
      }

    }
  }
}

// Iteratively expand dt1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h

/*double IterativeParabolicTimeParameterization::JerkfindT1( const double dq1,
                                                       const double dq2,
                                                       double dt1,
                                                       const double dt2,
                                                        
																											 const double jerk_max) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);
	double jerk = 2.0*(a-prev_a)/(dt1+dt2);

  while( std::abs( jerk) > jerk_max )
  {
    v1 = (dq1)/dt1;
    v2 = (dq2)/dt2;
    a = 2.0*(v2-v1)/(dt1+dt2);
    dt1 *= mult_factor;
		jerk = 2.0*(a-prev_a)/(dt1+dt2);
  }

  return dt1;
}
*/

/*double IterativeParabolicTimeParameterization::JerkfindT2(const double dq1,
                                                      const double dq2,
                                                      const double dt1,
                                                      double dt2,
                                                      
																											const double jerk_max, const double q1, const double q2, const double q3) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);
	double jerk = 2.0*(a-prev_a)/(dt1+dt2);
	printf("findT2\n");
  while( std::abs( jerk ) >  jerk_max)
  {
		/*printf("jerk: %f jerk_max: %f a: %f pre_a: %f dt1: %f dt2: %f q1: %f q2: %f q3: %f \n",jerk, jerk_max, a, prev_a, dt1, dt2, q1, q2, q3);
    v1 = (dq1)/dt1;
    v2 = (dq2)/dt2;
    a = 2.0*(v2-v1)/(dt1+dt2);
    dt2 *= mult_factor;
		jerk = 2.0*(a-prev_a)/(dt1+dt2);
  }

  return dt2;
}*/

double IterativeParabolicTimeParameterization::findT1( const double dq1,
                                                       const double dq2,
                                                       double dt1,
                                                       const double dt2,
                                                       const double a_max,
																												const int joint_number, const int index_check, const int point_number) const
{
	printf("joint_number: %d    ----------------------------------------------------------------", joint_number);
	printf("index_%d: \n", index_check);
	bool ignore = false;
  const double mult_factor_situation_1 = 1.01;
	const double mult_factor_situation_2 = 0.99;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);
	double jerk = (a-prev_a[joint_number])/(dt1);
	bool ignore_acc_process = false;
	
	printf("\n");
	printf("Before findT1 processing v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
	printf("\n");
	if (std::abs(jerk) > jerkLimit[joint_number] && std::abs(a) > std::abs(prev_a[joint_number]) && a*prev_a[joint_number] >=0 )
	{
		printf("findT1 jerk called\n");
  	while( (std::abs( a ) > a_max || std::abs( jerk ) >  jerkLimit[joint_number]) && ignore == false)
  	{
  	  v1 = (dq1)/dt1;
  	  v2 = (dq2)/dt2;
  	  if (index_check == point_number - 1)
			{
				a = (v2-v1)/(dt1);
			}
			else
			{
  	  	a = 2.0*(v2-v1)/(dt1+dt2);
			}
			jerk = (a-prev_a[joint_number])/(dt1);
			printf("v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
			if (std::abs(v2) < std::abs(v1))
			{
  	  	dt1 *= mult_factor_situation_1;
			}
			if (std::abs(v2) >= std::abs(v1))
			{
				ignore = true;
			}
  	}
	}
	else
	{
		printf("findT1 acceleration called\n");
  	while( std::abs( a ) > a_max && ignore_acc_process == false)
  	{
  	  v1 = (dq1)/dt1;
  	  v2 = (dq2)/dt2;
  	  if (index_check == point_number - 1)
			{
				a = (v2-v1)/(dt1);
			}
			else
			{
  	  	a = 2.0*(v2-v1)/(dt1+dt2);
			}
			
			printf("v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
			if (std::abs(v2) < std::abs(v1))
			{
  	  	dt1 *= mult_factor_situation_1;
			}
			if (std::abs(v2) >= std::abs(v1))
			{
				ignore_acc_process = true;
			}
	/*		if (std::abs(v2) < std::abs(v1))
			{
  	  	dt1 *= mult_factor_situation_1;
			}
			if (std::abs(v2) > std::abs(v1))
			{
				dt1 *= mult_factor_situation_2;
			}*/
  	}
	}
	if (index_check == point_number - 1)
	{
		last_a[joint_number] = a;
	}
	prev_a[joint_number] = a;
  return dt1;
}

/*double IterativeParabolicTimeParameterization::findT1( const double dq1,
                                                       const double dq2,
                                                       double dt1,
                                                       const double dt2,
                                                       const double a_max,
																												const int joint_number, const int index_check) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);
  	while( std::abs( a ) > a_max )
  	{
  	  v1 = (dq1)/dt1;
  	  v2 = (dq2)/dt2;
  	  a = 2.0*(v2-v1)/(dt1+dt2);
  	  dt1 *= mult_factor;
  	}
  return dt1;
}*/

double IterativeParabolicTimeParameterization::findT2(const double dq1,
                                                      const double dq2,
                                                      const double dt1,
                                                      double dt2,
                                                      const double a_max,
																											const int joint_number, const int index_check) const
{
	printf("joint_number: %d    ----------------------------------------------------------------", joint_number);
	printf("index_%d: \n", index_check);
  const double mult_factor_situation_1 = 1.01;
	const double mult_factor_situation_2 = 0.99;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);
	double jerk = (a-prev_a[joint_number])/(dt2);
	bool ignore = false;
	bool ignore_acc_process = false;
  //printf("findT2\n");
	printf("\n");
	printf("Before findT2 processing v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
	printf("\n");
	if (std::abs(jerk) > jerkLimit[joint_number] && std::abs(a) > std::abs(prev_a[joint_number]) && (a*prev_a[joint_number] >=0 ))
	{
		printf("findT2 jerk called\n");
  	while( (std::abs( a ) > a_max || std::abs( jerk ) >  jerkLimit[joint_number] ) && ignore == false)
  	{
			if (joint_number == 0)
			{
				//printf("jerk: %f jerk_max: %f a: %f pre_a: %f dt1: %f dt2: %f  \n",jerk, jerkLimit[joint_number], a, prev_a[joint_number], dt1, dt2);
			}
  	  v1 = (dq1)/dt1;
 	    v2 = (dq2)/dt2;
  	  if (index_check == 0)
			{
				a = (v2-v1)/(dt2);
			}
			else
			{
  	  	a = 2.0*(v2-v1)/(dt1+dt2);
			}
			jerk = (a-prev_a[joint_number])/(dt2);
			printf("v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
			if (std::abs(v2) > std::abs(v1))
			{
  	  	dt2 *= mult_factor_situation_1;
			}
			if (std::abs(v2) <= std::abs(v1))
			{
				ignore = true;
			}
  	}
	}
	else
	{
		printf("findT2 acceleration called\n");
  	while( std::abs( a ) > a_max && ignore_acc_process == false )
  	{
  	  v1 = (dq1)/dt1;
 	    v2 = (dq2)/dt2;
			if (index_check == 0)
			{
				a = (v2-v1)/(dt2);
			}
			else
			{
  	  	a = 2.0*(v2-v1)/(dt1+dt2);
			}
			printf("v1: %f v2: %f a: %f prev_a: %f jerk: %f dt1: %f dt2: %f \n", v1, v2, a, prev_a[joint_number], jerk, dt1, dt2);
			if (std::abs(v2) > std::abs(v1))
			{
  	  	dt2 *= mult_factor_situation_1;
			}
			if (std::abs(v2) <= std::abs(v1))
			{
				ignore_acc_process = true;
			}
			
/*			if (std::abs(v2) > std::abs(v1))
			{
  	  	dt2 *= mult_factor_situation_1;
			}
			if (std::abs(v2) < std::abs(v1))
			{
				dt2 *= mult_factor_situation_2;
			}*/
  	}
	}
	if (index_check == 0)
	{
		first_a[joint_number] = a;
	}
	prev_a[joint_number] = a;
  return dt2;
}

/*double IterativeParabolicTimeParameterization::findT2(const double dq1,
                                                      const double dq2,
                                                      const double dt1,
                                                      double dt2,
                                                      const double a_max,
																											const int joint_number, const int index_check) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);

  	while( std::abs( a ) > a_max )
  	{
  	  v1 = (dq1)/dt1;
 	    v2 = (dq2)/dt2;
  	  a = 2.0*(v2-v1)/(dt1+dt2);
    	dt2 *= mult_factor;
  	}

  return dt2;
}*/

namespace
{

// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.
void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory,
                      const std::vector<std::string>& active_joints,
                      const std::vector<double>& time_diff)
{
  
  double time_sum = 0.0;

  robot_state::RobotStatePtr prev_waypoint;
  robot_state::RobotStatePtr curr_waypoint;
  robot_state::RobotStatePtr next_waypoint;
  std::map<std::string, double> prev_state_values;
  std::map<std::string, double> curr_state_values;
  std::map<std::string, double> next_state_values;
  robot_state::JointState *jst;

  const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
  const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();

  unsigned int num_points = rob_trajectory.getWayPointCount();
  const unsigned int num_joints = group->getVariableCount();

  // Error check
  if(time_diff.size() < 1)
    return;

  rob_trajectory.setWayPointDurationFromPrevious(0, time_sum);

  // Times
  for (unsigned int i=1; i<num_points; ++i)
  {
    // Update the time between the waypoints in the robot_trajectory.
    rob_trajectory.setWayPointDurationFromPrevious(i, time_diff[i-1]);
  }

  // Return if there is only one point in the trajectory!
  if(num_points <= 1) return;

  // Accelerations
  for (unsigned int i=0; i<num_points; ++i)
  {
    curr_waypoint = rob_trajectory.getWayPointPtr(i);
    curr_waypoint->getStateValues(curr_state_values);

    if (i > 0)
    {
      prev_waypoint = rob_trajectory.getWayPointPtr(i-1);
      prev_waypoint->getStateValues(prev_state_values);
    }

    if (i < num_points-1)
    {
      next_waypoint = rob_trajectory.getWayPointPtr(i+1);
      next_waypoint->getStateValues(next_state_values);
    }


    for (unsigned int j=0; j<num_joints; ++j)
    {
      double q1;
      double q2;
      double q3;
      double dt1;
      double dt2;

      if(i==0)
      { // First point
        q1 = next_state_values[active_joints[j]];
        q2 = curr_state_values[active_joints[j]];
        q3 = next_state_values[active_joints[j]];

        dt1 = time_diff[i];
        dt2 = time_diff[i];
      }
      else if(i < num_points-1)
      { // middle points
        q1 = prev_state_values[active_joints[j]];
        q2 = curr_state_values[active_joints[j]];
        q3 = next_state_values[active_joints[j]];

        dt1 = time_diff[i-1];
        dt2 = time_diff[i];
      }
      else
      { // last point
        q1 = prev_state_values[active_joints[j]];
        q2 = curr_state_values[active_joints[j]];
        q3 = prev_state_values[active_joints[j]];

        dt1 = time_diff[i-1];
        dt2 = time_diff[i-1];
      }

      double v1, v2, a;

      bool start_velocity = false;
      if(dt1 == 0.0 || dt2 == 0.0) 
      {
        v1 = 0.0;
        v2 = 0.0;
        a = 0.0;
				if (j == 0)
				{
					printf("\n dt1 == 0 or dt2 == 0 \n");
				}
      } 
      else 
      {
        if(i==0 || i == num_points - 1)
				{
          const std::vector<double> &vel = curr_waypoint->getJointState(active_joints[j])->getVelocities();          
	  			if(!vel.empty())
	  			{
	    			start_velocity = true;
            v1 = vel[0];
          }
        }
        v1 = start_velocity ? v1 : (q2-q1)/dt1;
        //v2 = (q3-q2)/dt2;
        v2 = start_velocity ? v1 : (q3-q2)/dt2; // Needed to ensure continuous velocity for first point
        a = 2*(v2-v1)/(dt1+dt2);

      }

      jst = curr_waypoint->getJointState(active_joints[j]);
      // Update the velocities
      jst->getVelocities().resize(1);
      jst->getVelocities()[0] = (v2+v1)/2;
      // Update the accelerations
      jst->getAccelerations().resize(1);
			if (i == 0)
			{
				jst->getAccelerations()[0] = first_a[j];
			}
			else if (i == num_points - 1)
			{
				jst->getAccelerations()[0] = last_a[j];
			}
			else
			{
      	jst->getAccelerations()[0] = a;
			}
				/*if (j == 0)
				{
					printf("\n %f \n", a);
				}	*/
    }
  }
}
}

// Applies Acceleration constraints and jerk constraints    it's only partial jerk limited. And it is only good for point to point motion. If there is inflextion point, it's not good.
void IterativeParabolicTimeParameterization::applyAccelerationConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                                                          const std::vector<std::string>& active_joints,
                                                                          const std::vector<moveit_msgs::JointLimits>& limits,
                                                                          std::vector<double> & time_diff) const
{
  robot_state::RobotStatePtr prev_waypoint;
  robot_state::RobotStatePtr curr_waypoint;
  robot_state::RobotStatePtr next_waypoint;
  std::map<std::string, double> prev_state_values;
  std::map<std::string, double> curr_state_values;
  std::map<std::string, double> next_state_values;

  const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
  const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();

  const unsigned int num_points = rob_trajectory.getWayPointCount();
  const unsigned int num_joints = group->getVariableCount();
	//printf("\n numpoints: %d, numjoints: %d \n", num_points, num_joints);
  int num_updates = 0;
  int iteration = 0;
  bool backwards = false;
  double q1;
  double q2;
  double q3;
  double dt1;
  double dt2;
  double v1;
  double v2;
  double a;

  do
  {
    num_updates = 0;
    iteration++;
		printf("\n iteration: %d \n", iteration);
    // In this case we iterate through the joints on the outer loop.
    // This is so that any time interval increases have a chance to get propogated through the trajectory
    for (unsigned int j = 0; j < num_joints ; ++j)
    {
      // Loop forwards, then backwards
      for( int count=0; count<2; count++)
      {
				for (int i = 0; i<7; i++)
				{
					prev_a[i]=0;
				}
				//prev_a = 0;
        for (unsigned int i=0; i<num_points-1; i++)
        {
          unsigned int index = i;
					applyJerkLimit = true;
					//printf("%d ",index);
          if(backwards)
          {
            index = (num_points-1)-i;
          }

          curr_waypoint = rob_trajectory.getWayPointPtr(index);
          curr_waypoint->getStateValues(curr_state_values);

          if (index > 0)
          {
            prev_waypoint = rob_trajectory.getWayPointPtr(index-1);
            prev_waypoint->getStateValues(prev_state_values);
          }

          if (index < num_points-1)
          {
            next_waypoint = rob_trajectory.getWayPointPtr(index+1);
            next_waypoint->getStateValues(next_state_values);
          }

          // Get acceleration limits
          double a_max = 1.0;
          if( limits[j].has_acceleration_limits )
          {
            a_max = limits[j].max_acceleration;
          }

          if(index==0)
          {     // First point
            q1 = next_state_values[active_joints[j]];
            q2 = curr_state_values[active_joints[j]];
            q3 = next_state_values[active_joints[j]];

            dt1 = time_diff[index];
            dt2 = time_diff[index];
            assert(!backwards);
          }
          else if(index < num_points-1)
          { // middle points
            q1 = prev_state_values[active_joints[j]];
            q2 = curr_state_values[active_joints[j]];
            q3 = next_state_values[active_joints[j]];

            dt1 = time_diff[index-1];
            dt2 = time_diff[index];
          }
          else
          { // last point - careful, there are only numpoints-1 time intervals
            q1 = prev_state_values[active_joints[j]];
            q2 = curr_state_values[active_joints[j]];
            q3 = prev_state_values[active_joints[j]];

            dt1 = time_diff[index-1];
            dt2 = time_diff[index-1];
            assert(backwards);
          }

          if(dt1 == 0.0 || dt2 == 0.0) {
            v1 = 0.0;
            v2 = 0.0;
            a = 0.0;

							printf("1      dt1 == 0 || dt2 == 0 \n");

          } else {
            bool start_velocity = false;
            if(index==0 || index==num_points-1)
            {
              const std::vector<double> &vel = curr_waypoint->getJointState(active_joints[j])->getVelocities();          
              if(!vel.empty())
              {
                start_velocity = true;
								if (index == 0)
								{
                	v1 = 0;
								}
								if (index == num_points - 1)
								{
									v2 = 0;
								}

              }
            }
            v1 = (start_velocity && index==0) ? v1 : (q2-q1)/dt1;
            v2 = (start_velocity && index==num_points - 1) ? v2 : (q3-q2)/dt2;
            a = 2*(v2-v1)/(dt1+dt2);
          }
					a_profile_before[index] = a;
					time_diff_before[index] = time_diff[index];
					double tempJerk = 0;
					if (!backwards)
					{
						tempJerk = (a-prev_a[j])/(dt2);
					}
					else
					{
						tempJerk = (a-prev_a[j])/(dt1);
					}
					if (!backwards && index == num_points-1)
					{
						applyJerkLimit = false;	
					}
					if (backwards && index == 0)
					{
						applyJerkLimit = false;	
					}
					
          if( (std::abs( a ) > a_max + ROUNDING_THRESHOLD || std::abs(tempJerk) > jerkLimit[j] ) && applyJerkLimit == true)
          {
            if(!backwards)
            {
            
							double temp_findT2;
							if (!backwards && index == 0)
							{
								temp_findT2 = findT2( 0, q3-q2, dt1, dt2, a_max, j, index);
							}
							else
							{		
								temp_findT2 = findT2( q2-q1, q3-q2, dt1, dt2, a_max, j, index);
							}
							dt2 = std::min( dt2+5, temp_findT2 );

              time_diff[index] = dt2;
            }
            else
            {
							double temp_findT1;
							if (backwards && index == num_points - 1)
							{
								temp_findT1 = findT1( q2-q1, 0, dt1, dt2, a_max, j, index, num_points);
							}
							else
							{		
								temp_findT1 = findT1( q2-q1, q3-q2, dt1, dt2, a_max, j, index, num_points);
							}
              dt1 = std::min( dt1+5, temp_findT1 );
              time_diff[index-1] = dt1;
            }
            num_updates++;
						
            if(dt1 == 0.0 || dt2 == 0.0) {
              v1 = 0.0;
              v2 = 0.0;
              a = 0.0;

								printf("2      dt1 == 0 || dt2 == 0 \n");

            } else {
							if (index == 0 && !backwards)
							{
						 		v1 = 0;
              	v2 = (q3-q2)/dt2;
              	a = (v2-v1)/(dt2);
							}
							else if (index == num_points - 1 && backwards)
							{
						 		v1 = (q2-q1)/dt1;
              	v2 = 0;
              	a = (v2-v1)/(dt1);						
							}
							else
							{
              	v1 = (q2-q1)/dt1;
              	v2 = (q3-q2)/dt2;
              	a = 2*(v2-v1)/(dt1+dt2);
							}
						
            }
          }
					

					
					a_profile[index] = a;

        }

					if (!backwards)
					{
						printf("forward  \n");
					}
					else
					{	
						printf("backward \n");
					}
					printf("before processing   \n");
					for (int i=0; i<num_points	; i++)
					{
						printf("%f ", a_profile_before[i]);
					}
					printf("\n");
					for (int i=0; i<num_points-1	; i++)
					{
						printf("%f ", time_diff_before[i]);
					}
					printf("\n");
					printf("after processing   \n");
					for (int i=0; i<num_points	; i++)
					{
						printf("%f ", a_profile[i]);
					}
					printf("\n");
					for (int i=0; i<num_points-1	; i++)
					{
						printf("%f ", time_diff[i]);
					}
					printf("\n");
					printf("-------------------------------------------\n");
				

        backwards = !backwards;
      }
    }
  } while(num_updates > 0 && iteration < 1);
}




bool IterativeParabolicTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory) const
{




  bool success = true;
  robot_state::RobotStatePtr curr_waypoint;
  const robot_state::JointState *jst;

	ros::param::get("/robot_description_planning/velocity_limit_factor", velocity_limit_factor);
	
	printf("\nvelocity_limit_factor:                   %f   \n",velocity_limit_factor);
  if (trajectory.empty())
    return true;

  const robot_model::JointModelGroup *group = trajectory.getGroup();
  if (!group)
  {
    logError("It looks like the planner did not set the group the plan was computed for");
    return false;
  }
  
  const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() != 1)
    {
      logWarn("Time parametrization works for single-dof joints only");
      return false;
    }
  
  const std::vector<moveit_msgs::JointLimits> &limits = trajectory.getGroup()->getVariableLimits();
  const std::vector<std::string> &active_joints = group->getJointModelNames();
  
  // this lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  const std::size_t num_points = trajectory.getWayPointCount();
	printf("number of way points: %d \n", num_points);
  std::vector<double> time_diff(num_points-1, 0.0);       // the time difference between adjacent points
	printf("\n");

	//applyRML(trajectory, active_joints, limits, time_diff);
  applyVelocityConstraints(trajectory, active_joints, limits, time_diff);
  applyAccelerationConstraints(trajectory, active_joints, limits, time_diff);
  updateTrajectory(trajectory, active_joints, time_diff);
  return success;
}

}
