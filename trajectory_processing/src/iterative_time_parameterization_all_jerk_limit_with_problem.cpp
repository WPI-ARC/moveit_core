#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/JointLimits.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>

namespace trajectory_processing
{

static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 1.0;
static const double ROUNDING_THRESHOLD = 0.01;
double previous_a[7];

double fixed_velocity[7][200];
double j_max[7]={4.5, 4.52, 4.5, 4.5, 4.5, 4.5, 4.5};
double a_profile[7][200];

IterativeParabolicTimeParameterization::IterativeParabolicTimeParameterization(unsigned int max_iterations,
                                                                               double max_time_change_per_it)
  : max_iterations_(max_iterations),
    max_time_change_per_it_(max_time_change_per_it)
{}

IterativeParabolicTimeParameterization::~IterativeParabolicTimeParameterization()
{}

void IterativeParabolicTimeParameterization::printPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const
{
  logDebug( " time [%i]= %f",i,point.time_from_start.toSec());
  if(point.positions.size() >= 7 )
  {
    logDebug(" pos_ [%i]= %f %f %f %f %f %f %f",i,
             point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
  }
  if(point.velocities.size() >= 7 )
  {
    logDebug(" vel_ [%i]= %f %f %f %f %f %f %f",i,
             point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
  }
  if(point.accelerations.size() >= 7 )
  {
    logDebug(" acc_ [%i]= %f %f %f %f %f %f %f",i,
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
        v_max = limits[joint].max_velocity;
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
double IterativeParabolicTimeParameterization::findT1( const double dq1,
                                                       const double dq2,
                                                       double dt1,
                                                       const double dt2,
                                                       const double a_max) const
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
}

double IterativeParabolicTimeParameterization::findT2(const double dq1,
                                                      const double dq2,
                                                      const double dt1,
                                                      double dt2,
                                                      const double a_max) const
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
}

namespace
{

// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.


void storeFixedVelocity(robot_trajectory::RobotTrajectory& rob_trajectory, const std::vector<std::string>& active_joints, const std::vector<double>& time_diff)
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
      }
      else
      {
        if(i==0)
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

      fixed_velocity[j][i] = (v2+v1)/2;

    }
  }
}

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
      }
      else
      {
        if(i==0)
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
      jst->getVelocities()[0] = fixed_velocity[j][i];
      // Update the accelerations
      jst->getAccelerations().resize(1);
      jst->getAccelerations()[0] = a_profile[j][i];
    }
  }
}
}

// Applies Acceleration constraints
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

    // In this case we iterate through the joints on the outer loop.
    // This is so that any time interval increases have a chance to get propogated through the trajectory
    for (unsigned int j = 0; j < num_joints ; ++j)
    {
      // Loop forwards, then backwards
      for( int count=0; count<2; count++)
      {
        for (unsigned int i=0; i<num_points-1; ++i)
        {
          unsigned int index = i;
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
          { // First point
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
          } else {
            bool start_velocity = false;
            if(index==0)
            {
              const std::vector<double> &vel = curr_waypoint->getJointState(active_joints[j])->getVelocities();
              if(!vel.empty())
              {
                start_velocity = true;
                v1 = vel[0];
              }
            }
            v1 = start_velocity ? v1 : (q2-q1)/dt1;
            v2 = (q3-q2)/dt2;
            a = 2*(v2-v1)/(dt1+dt2);
          }

          if( std::abs( a ) > a_max + ROUNDING_THRESHOLD )
          {
            if(!backwards)
            {
              dt2 = std::min( dt2+max_time_change_per_it_, findT2( q2-q1, q3-q2, dt1, dt2, a_max) );
              time_diff[index] = dt2;
            }
            else
            {
              dt1 = std::min( dt1+max_time_change_per_it_, findT1( q2-q1, q3-q2, dt1, dt2, a_max) );
              time_diff[index-1] = dt1;
            }
            num_updates++;

            if(dt1 == 0.0 || dt2 == 0.0) {
              v1 = 0.0;
              v2 = 0.0;
              a = 0.0;
            } else {
              v1 = (q2-q1)/dt1;
              v2 = (q3-q2)/dt2;
              a = 2*(v2-v1)/(dt1+dt2);
            }
          }
        }
        backwards = !backwards;
      }
    }
    //logDebug("applyAcceleration: num_updates=%i", num_updates);
  } while(num_updates > 0 && iteration < max_iterations_);
}


void IterativeParabolicTimeParameterization::applyJerkConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
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
	double jerk;

    for (unsigned int j = 0; j < num_joints ; ++j)
    {
      printf("joint_number: %d \n", j );

        for (unsigned int i=0; i<num_points; ++i)
        {
          unsigned int index = i;


          if(index==0)
          { // First point
						a = 0;
          }
          else if(index < num_points-1)
          { // middle points
						v1 = fixed_velocity[j][index-1];
						v2 = fixed_velocity[j][index+1];
						dt1 = time_diff[index-1];
						dt2 = time_diff[index];
						a = 2*(v2-v1)/(dt1+dt2);
						jerk = (a - previous_a[j])/dt1;

          }
          else
          { // last point - careful, there are only numpoints-1 time intervals
						a = 0;
						dt1 = time_diff[index-1];
						jerk = (a - previous_a[j])/dt1;
          }



					bool applyJerkLimit;
					if (index == 0)
					{
						applyJerkLimit == false;
					}
					else
					{
						applyJerkLimit == true;
					}

								if (j == 0)
								{
									printf("index: %d v1: %f v2: %f prev_a: %f a: %f jerk: %f dt1: %f dt2: %f j_max: %f \n", index, v1, v2, previous_a[j], a, jerk, dt1, dt2, j_max[j] );
								}

          if( std::abs( jerk ) > j_max[j] + ROUNDING_THRESHOLD)
          {
						printf("apply jerk !\n");
  					const double mult_factor = 1.01;
						bool keep_a = false;
						printf("\n");
  					while( std::abs( jerk ) > j_max[j] + ROUNDING_THRESHOLD)
  					{
							if (std::abs( a ) > std::abs( previous_a[j] ))
							{
								keep_a = false;
								a = 2.0*(v2-v1)/(dt1+dt2);
								jerk = (a - previous_a[j])/dt1;
    						dt1 *= mult_factor;
								if (j == 0)
								{
									printf("index: %d v1: %f v2: %f prev_a: %f a: %f jerk: %f dt1: %f dt2: %f \n", index, v1, v2, previous_a[j], a, jerk, dt1, dt2 );
								}
							}
							else
							{
								keep_a = true;
								jerk = (a - previous_a[j])/dt1;
								dt1 *= mult_factor;
								if (j == 0)
								{
									printf("index: %d v1: %f v2: %f prev_a: %f a: %f jerk: %f dt1: %f dt2: %f \n", index, v1, v2, previous_a[j], a, jerk, dt1, dt2 );
								}
							}
  					}
            time_diff[index-1] = dt1;

						if (keep_a == false)
						{
            	a = 2.0*(v2-v1)/(dt1+dt2);
						}
						else 
						{
							printf("keep_a : %f \n", a);
						}
						
          }
					a_profile[j][i] = a;
					previous_a[j] = a;
        }
    }


}

bool IterativeParabolicTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory) const
{
  bool success = true;
  robot_state::RobotStatePtr curr_waypoint;
  const robot_state::JointState *jst;

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

  std::vector<double> time_diff(num_points-1, 0.0); // the time difference between adjacent points

  applyVelocityConstraints(trajectory, active_joints, limits, time_diff);
  applyAccelerationConstraints(trajectory, active_joints, limits, time_diff);
	storeFixedVelocity(trajectory, active_joints, time_diff);
	applyJerkConstraints(trajectory, active_joints, limits, time_diff);
	printf("\nvelocity \n");
	for (int i = 0; i < num_points; i++)
	{
		printf("%f ", fixed_velocity[0][i]);
	}
  printf("\nvelocity \n");
  updateTrajectory(trajectory, active_joints, time_diff);
  return success;
}

}
