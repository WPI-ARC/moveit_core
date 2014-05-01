/*********************************************************************
00002  * Software License Agreement (BSD License)
00003  *
00004  *  Copyright (c) 2009, Willow Garage, Inc.
00005  *  All rights reserved.
00006  *
00007  *  Redistribution and use in source and binary forms, with or without
00008  *  modification, are permitted provided that the following conditions
00009  *  are met:
00010  *
00011  *   * Redistributions of source code must retain the above copyright
00012  *     notice, this list of conditions and the following disclaimer.
00013  *   * Redistributions in binary form must reproduce the above
00014  *     copyright notice, this list of conditions and the following
00015  *     disclaimer in the documentation and/or other materials provided
00016  *     with the distribution.
00017  *   * Neither the name of the Willow Garage nor the names of its
00018  *     contributors may be used to endorse or promote products derived
00019  *     from this software without specific prior written permission.
00020  *
00021  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
00022  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
00023  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
00024  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
00025  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
00026  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
00027  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00028  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
00029  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
00030  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
00031  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00032  *  POSSIBILITY OF SUCH DAMAGE.
00033  *********************************************************************/
00034 
00035 /* Author: Ken Anderson */
00036 
00037 #include <moveit/trajectory_processing/iterative_time_parameterization.h>
00038 #include <moveit_msgs/JointLimits.h>
00039 #include <console_bridge/console.h>
00040 #include <moveit/robot_state/conversions.h>
00041 
00042 namespace trajectory_processing
00043 {
00044 
00045 static const double DEFAULT_VEL_MAX = 1.0;
00046 static const double DEFAULT_ACCEL_MAX = 1.0;
00047 static const double ROUNDING_THRESHOLD = 0.01;
00048 
00049 IterativeParabolicTimeParameterization::IterativeParabolicTimeParameterization(unsigned int max_iterations,
00050                                                                                double max_time_change_per_it)
00051   : max_iterations_(max_iterations),
00052     max_time_change_per_it_(max_time_change_per_it)
00053 {}
00054 
00055 IterativeParabolicTimeParameterization::~IterativeParabolicTimeParameterization()
00056 {}
00057 
00058 void IterativeParabolicTimeParameterization::printPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const
00059 {
00060   logDebug(  " time   [%i]= %f",i,point.time_from_start.toSec());
00061   if(point.positions.size() >= 7 )
00062   {
00063     logDebug(" pos_   [%i]= %f %f %f %f %f %f %f",i,
00064              point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
00065   }
00066   if(point.velocities.size() >= 7 )
00067   {
00068     logDebug("  vel_  [%i]= %f %f %f %f %f %f %f",i,
00069              point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
00070   }
00071   if(point.accelerations.size() >= 7 )
00072   {
00073     logDebug("   acc_ [%i]= %f %f %f %f %f %f %f",i,
00074              point.accelerations[0],point.accelerations[1],point.accelerations[2],point.accelerations[3],point.accelerations[4],point.accelerations[5],point.accelerations[6]);
00075   }
00076 }
00077 
00078 void IterativeParabolicTimeParameterization::printStats(const trajectory_msgs::JointTrajectory& trajectory,
00079                                                         const std::vector<moveit_msgs::JointLimits>& limits) const
00080 {
00081   logDebug("jointNames= %s %s %s %s %s %s %s",
00082            limits[0].joint_name.c_str(),limits[1].joint_name.c_str(),limits[2].joint_name.c_str(),
00083            limits[3].joint_name.c_str(),limits[4].joint_name.c_str(),limits[5].joint_name.c_str(),
00084            limits[6].joint_name.c_str());
00085   logDebug("maxVelocities= %f %f %f %f %f %f %f",
00086            limits[0].max_velocity,limits[1].max_velocity,limits[2].max_velocity,
00087            limits[3].max_velocity,limits[4].max_velocity,limits[5].max_velocity,
00088            limits[6].max_velocity);
00089   logDebug("maxAccelerations= %f %f %f %f %f %f %f",
00090            limits[0].max_acceleration,limits[1].max_acceleration,limits[2].max_acceleration,
00091            limits[3].max_acceleration,limits[4].max_acceleration,limits[5].max_acceleration,
00092            limits[6].max_acceleration);
00093   // for every point in time:
00094   for (unsigned int i=0; i<trajectory.points.size(); ++i)
00095   {
00096     const trajectory_msgs::JointTrajectoryPoint& point = trajectory.points[i];
00097     printPoint(point, i);
00098   }
00099 }
00100 
00101 // Applies velocity
00102 void IterativeParabolicTimeParameterization::applyVelocityConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
00103                                                                       const std::vector<std::string>& active_joints,
00104                                                                       const std::vector<moveit_msgs::JointLimits>& limits,
00105                                                                       std::vector<double> &time_diff) const
00106 {
00107 
00108   robot_state::RobotStatePtr curr_waypoint;
00109   robot_state::RobotStatePtr next_waypoint;
00110   std::map<std::string, double> curr_state_values;
00111   std::map<std::string, double> next_state_values;
00112 
00113   const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
00114   const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
00115 
00116   const unsigned int num_points = rob_trajectory.getWayPointCount();
00117   const unsigned int num_joints = group->getVariableCount();
00118 
00119   for( unsigned int i=0; i<num_points-1; ++i )
00120   {
00121     curr_waypoint = rob_trajectory.getWayPointPtr(i);
00122     next_waypoint = rob_trajectory.getWayPointPtr(i+1);
00123 
00124     curr_waypoint->getStateValues(curr_state_values);
00125     next_waypoint->getStateValues(next_state_values);
00126 
00127     for (unsigned int joint=0; joint < active_joints.size(); joint++)
00128     {
00129       double v_max = 1.0;
00130 
00131       if( limits[joint].has_velocity_limits )
00132       {
00133         v_max = limits[joint].max_velocity;
00134       }
00135       const double dq1 = curr_state_values[active_joints[joint]];
00136       const double dq2 = next_state_values[active_joints[joint]];
00137       const double t_min = std::abs(dq2-dq1) / v_max;
00138       if( t_min > time_diff[i] )
00139       {
00140         time_diff[i] = t_min;
00141       }
00142 
00143     }
00144   }
00145 }
00146 
00147 // Iteratively expand dt1 interval by a constant factor until within acceleration constraint
00148 // In the future we may want to solve to quadratic equation to get the exact timing interval.
00149 // To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h
00150 double IterativeParabolicTimeParameterization::findT1( const double dq1,
00151                                                        const double dq2,
00152                                                        double dt1,
00153                                                        const double dt2,
00154                                                        const double a_max) const
00155 {
00156   const double mult_factor = 1.01;
00157   double v1 = (dq1)/dt1;
00158   double v2 = (dq2)/dt2;
00159   double a = 2.0*(v2-v1)/(dt1+dt2);
00160 
00161   while( std::abs( a ) > a_max )
00162   {
00163     v1 = (dq1)/dt1;
00164     v2 = (dq2)/dt2;
00165     a = 2.0*(v2-v1)/(dt1+dt2);
00166     dt1 *= mult_factor;
00167   }
00168 
00169   return dt1;
00170 }
00171 
00172 double IterativeParabolicTimeParameterization::findT2(const double dq1,
00173                                                       const double dq2,
00174                                                       const double dt1,
00175                                                       double dt2,
00176                                                       const double a_max) const
00177 {
00178   const double mult_factor = 1.01;
00179   double v1 = (dq1)/dt1;
00180   double v2 = (dq2)/dt2;
00181   double a = 2.0*(v2-v1)/(dt1+dt2);
00182 
00183   while( std::abs( a ) > a_max )
00184   {
00185     v1 = (dq1)/dt1;
00186     v2 = (dq2)/dt2;
00187     a = 2.0*(v2-v1)/(dt1+dt2);
00188     dt2 *= mult_factor;
00189   }
00190 
00191   return dt2;
00192 }
00193 
00194 namespace
00195 {
00196 
00197 // Takes the time differences, and updates the timestamps, velocities and accelerations
00198 // in the trajectory.
00199 void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory,
00200                       const std::vector<std::string>& active_joints,
00201                       const std::vector<double>& time_diff)
00202 {
00203 
00204   double time_sum = 0.0;
00205 
00206   robot_state::RobotStatePtr prev_waypoint;
00207   robot_state::RobotStatePtr curr_waypoint;
00208   robot_state::RobotStatePtr next_waypoint;
00209   std::map<std::string, double> prev_state_values;
00210   std::map<std::string, double> curr_state_values;
00211   std::map<std::string, double> next_state_values;
00212   robot_state::JointState *jst;
00213 
00214   const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
00215   const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
00216 
00217   unsigned int num_points = rob_trajectory.getWayPointCount();
00218   const unsigned int num_joints = group->getVariableCount();
00219 
00220   // Error check
00221   if(time_diff.size() < 1)
00222     return;
00223 
00224   rob_trajectory.setWayPointDurationFromPrevious(0, time_sum);
00225 
00226   // Times
00227   for (unsigned int i=1; i<num_points; ++i)
00228   {
00229     // Update the time between the waypoints in the robot_trajectory.
00230     rob_trajectory.setWayPointDurationFromPrevious(i, time_diff[i-1]);
00231   }
00232 
00233   // Return if there is only one point in the trajectory!
00234   if(num_points <= 1) return;
00235 
00236   // Accelerations
00237   for (unsigned int i=0; i<num_points; ++i)
00238   {
00239     curr_waypoint = rob_trajectory.getWayPointPtr(i);
00240     curr_waypoint->getStateValues(curr_state_values);
00241 
00242     if (i > 0)
00243     {
00244       prev_waypoint = rob_trajectory.getWayPointPtr(i-1);
00245       prev_waypoint->getStateValues(prev_state_values);
00246     }
00247 
00248     if (i < num_points-1)
00249     {
00250       next_waypoint = rob_trajectory.getWayPointPtr(i+1);
00251       next_waypoint->getStateValues(next_state_values);
00252     }
00253 
00254 
00255     for (unsigned int j=0; j<num_joints; ++j)
00256     {
00257       double q1;
00258       double q2;
00259       double q3;
00260       double dt1;
00261       double dt2;
00262 
00263       if(i==0)
00264       { // First point
00265         q1 = next_state_values[active_joints[j]];
00266         q2 = curr_state_values[active_joints[j]];
00267         q3 = next_state_values[active_joints[j]];
00268 
00269         dt1 = time_diff[i];
00270         dt2 = time_diff[i];
00271       }
00272       else if(i < num_points-1)
00273       { // middle points
00274         q1 = prev_state_values[active_joints[j]];
00275         q2 = curr_state_values[active_joints[j]];
00276         q3 = next_state_values[active_joints[j]];
00277 
00278         dt1 = time_diff[i-1];
00279         dt2 = time_diff[i];
00280       }
00281       else
00282       { // last point
00283         q1 = prev_state_values[active_joints[j]];
00284         q2 = curr_state_values[active_joints[j]];
00285         q3 = prev_state_values[active_joints[j]];
00286 
00287         dt1 = time_diff[i-1];
00288         dt2 = time_diff[i-1];
00289       }
00290 
00291       double v1, v2, a;
00292 
00293       bool start_velocity = false;
00294       if(dt1 == 0.0 || dt2 == 0.0)
00295       {
00296         v1 = 0.0;
00297         v2 = 0.0;
00298         a = 0.0;
00299       }
00300       else
00301       {
00302         if(i==0)
00303     {
00304           const std::vector<double> &vel = curr_waypoint->getJointState(active_joints[j])->getVelocities();
00305       if(!vel.empty())
00306       {
00307         start_velocity = true;
00308             v1 = vel[0];
00309           }
00310         }
00311         v1 = start_velocity ? v1 : (q2-q1)/dt1;
00312         //v2 = (q3-q2)/dt2;
00313         v2 = start_velocity ? v1 : (q3-q2)/dt2; // Needed to ensure continuous velocity for first point
00314         a = 2*(v2-v1)/(dt1+dt2);
00315       }
00316 
00317       jst = curr_waypoint->getJointState(active_joints[j]);
00318       // Update the velocities
00319       jst->getVelocities().resize(1);
00320       jst->getVelocities()[0] = (v2+v1)/2;
00321       // Update the accelerations
00322       jst->getAccelerations().resize(1);
00323       jst->getAccelerations()[0] = a;
00324     }
00325   }
00326 }
00327 }
00328 
00329 // Applies Acceleration constraints
00330 void IterativeParabolicTimeParameterization::applyAccelerationConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
00331                                                                           const std::vector<std::string>& active_joints,
00332                                                                           const std::vector<moveit_msgs::JointLimits>& limits,
00333                                                                           std::vector<double> & time_diff) const
00334 {
00335   robot_state::RobotStatePtr prev_waypoint;
00336   robot_state::RobotStatePtr curr_waypoint;
00337   robot_state::RobotStatePtr next_waypoint;
00338   std::map<std::string, double> prev_state_values;
00339   std::map<std::string, double> curr_state_values;
00340   std::map<std::string, double> next_state_values;
00341 
00342   const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
00343   const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
00344 
00345   const unsigned int num_points = rob_trajectory.getWayPointCount();
00346   const unsigned int num_joints = group->getVariableCount();
00347   int num_updates = 0;
00348   int iteration = 0;
00349   bool backwards = false;
00350   double q1;
00351   double q2;
00352   double q3;
00353   double dt1;
00354   double dt2;
00355   double v1;
00356   double v2;
00357   double a;
00358 
00359   do
00360   {
00361     num_updates = 0;
00362     iteration++;
00363 
00364     // In this case we iterate through the joints on the outer loop.
00365     // This is so that any time interval increases have a chance to get propogated through the trajectory
00366     for (unsigned int j = 0; j < num_joints ; ++j)
00367     {
00368       // Loop forwards, then backwards
00369       for( int count=0; count<2; count++)
00370       {
00371         for (unsigned int i=0; i<num_points-1; ++i)
00372         {
00373           unsigned int index = i;
00374           if(backwards)
00375           {
00376             index = (num_points-1)-i;
00377           }
00378 
00379           curr_waypoint = rob_trajectory.getWayPointPtr(index);
00380           curr_waypoint->getStateValues(curr_state_values);
00381 
00382           if (index > 0)
00383           {
00384             prev_waypoint = rob_trajectory.getWayPointPtr(index-1);
00385             prev_waypoint->getStateValues(prev_state_values);
00386           }
00387 
00388           if (index < num_points-1)
00389           {
00390             next_waypoint = rob_trajectory.getWayPointPtr(index+1);
00391             next_waypoint->getStateValues(next_state_values);
00392           }
00393 
00394           // Get acceleration limits
00395           double a_max = 1.0;
00396           if( limits[j].has_acceleration_limits )
00397           {
00398             a_max = limits[j].max_acceleration;
00399           }
00400 
00401           if(index==0)
00402           {     // First point
00403             q1 = next_state_values[active_joints[j]];
00404             q2 = curr_state_values[active_joints[j]];
00405             q3 = next_state_values[active_joints[j]];
00406 
00407             dt1 = time_diff[index];
00408             dt2 = time_diff[index];
00409             assert(!backwards);
00410           }
00411           else if(index < num_points-1)
00412           { // middle points
00413             q1 = prev_state_values[active_joints[j]];
00414             q2 = curr_state_values[active_joints[j]];
00415             q3 = next_state_values[active_joints[j]];
00416 
00417             dt1 = time_diff[index-1];
00418             dt2 = time_diff[index];
00419           }
00420           else
00421           { // last point - careful, there are only numpoints-1 time intervals
00422             q1 = prev_state_values[active_joints[j]];
00423             q2 = curr_state_values[active_joints[j]];
00424             q3 = prev_state_values[active_joints[j]];
00425 
00426             dt1 = time_diff[index-1];
00427             dt2 = time_diff[index-1];
00428             assert(backwards);
00429           }
00430 
00431           if(dt1 == 0.0 || dt2 == 0.0) {
00432             v1 = 0.0;
00433             v2 = 0.0;
00434             a = 0.0;
00435           } else {
00436             bool start_velocity = false;
00437             if(index==0)
00438             {
00439               const std::vector<double> &vel = curr_waypoint->getJointState(active_joints[j])->getVelocities();
00440               if(!vel.empty())
00441               {
00442                 start_velocity = true;
00443                 v1 = vel[0];
00444               }
00445             }
00446             v1 = start_velocity ? v1 : (q2-q1)/dt1;
00447             v2 = (q3-q2)/dt2;
00448             a = 2*(v2-v1)/(dt1+dt2);
00449           }
00450 
00451           if( std::abs( a ) > a_max + ROUNDING_THRESHOLD )
00452           {
00453             if(!backwards)
00454             {
00455               dt2 = std::min( dt2+max_time_change_per_it_, findT2( q2-q1, q3-q2, dt1, dt2, a_max) );
00456               time_diff[index] = dt2;
00457             }
00458             else
00459             {
00460               dt1 = std::min( dt1+max_time_change_per_it_, findT1( q2-q1, q3-q2, dt1, dt2, a_max) );
00461               time_diff[index-1] = dt1;
00462             }
00463             num_updates++;
00464 
00465             if(dt1 == 0.0 || dt2 == 0.0) {
00466               v1 = 0.0;
00467               v2 = 0.0;
00468               a = 0.0;
00469             } else {
00470               v1 = (q2-q1)/dt1;
00471               v2 = (q3-q2)/dt2;
00472               a = 2*(v2-v1)/(dt1+dt2);
00473             }
00474           }
00475         }
00476         backwards = !backwards;
00477       }
00478     }
00479     //logDebug("applyAcceleration: num_updates=%i", num_updates);
00480   } while(num_updates > 0 && iteration < max_iterations_);
00481 }
00482 
00483 bool IterativeParabolicTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory) const
00484 {
00485   bool success = true;
00486   robot_state::RobotStatePtr curr_waypoint;
00487   const robot_state::JointState *jst;
00488 
00489   if (trajectory.empty())
00490     return true;
00491 
00492   const robot_model::JointModelGroup *group = trajectory.getGroup();
00493   if (!group)
00494   {
00495     logError("It looks like the planner did not set the group the plan was computed for");
00496     return false;
00497   }
00498 
00499   const std::vector<const robot_model::JointModel*> &jnt = group->getJointModels();
00500   for (std::size_t i = 0 ; i < jnt.size() ; ++i)
00501     if (jnt[i]->getVariableCount() != 1)
00502     {
00503       logWarn("Time parametrization works for single-dof joints only");
00504       return false;
00505     }
00506 
00507   const std::vector<moveit_msgs::JointLimits> &limits = trajectory.getGroup()->getVariableLimits();
00508   const std::vector<std::string> &active_joints = group->getJointModelNames();
00509 
00510   // this lib does not actually work properly when angles wrap around, so we need to unwind the path first
00511   trajectory.unwind();
00512 
00513   const std::size_t num_points = trajectory.getWayPointCount();
00514 
00515   std::vector<double> time_diff(num_points-1, 0.0);       // the time difference between adjacent points
00516 
00517   applyVelocityConstraints(trajectory, active_joints, limits, time_diff);
00518   applyAccelerationConstraints(trajectory, active_joints, limits, time_diff);
00519 
00520   updateTrajectory(trajectory, active_joints, time_diff);
00521   return success;
00522 }
00523 
00524 }
