#ifndef MOVEIT_TRAJECTORY_PROCESSING_ITERATIVE_PARABOLIC_SMOOTHER_
#define MOVEIT_TRAJECTORY_PROCESSING_ITERATIVE_PARABOLIC_SMOOTHER_

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{

/// \brief This class modifies the timestamps of a trajectory to respect
/// velocity and acceleration constraints.
class IterativeParabolicTimeParameterization
{
public:
  IterativeParabolicTimeParameterization(unsigned int max_iterations = 100,
                                         double max_time_change_per_it = .01);
  ~IterativeParabolicTimeParameterization();

  bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory) const;

private:

  unsigned int max_iterations_; /// @brief maximum number of iterations to find solution
  double max_time_change_per_it_; /// @brief maximum allowed time change per iteration in seconds

  void applyVelocityConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                const std::vector<std::string>& active_joints,
                                const std::vector<moveit_msgs::JointLimits>& limits,
                                std::vector<double> &time_diff) const;

  void applyAccelerationConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                    const std::vector<std::string>& active_joints,
                                    const std::vector<moveit_msgs::JointLimits>& limits,
                                    std::vector<double> & time_diff) const;

  void applyJerkConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                    const std::vector<std::string>& active_joints,
                                    const std::vector<moveit_msgs::JointLimits>& limits,
                                    std::vector<double> & time_diff) const;

  double findT1( const double d1, const double d2, double t1, const double t2, const double a_max) const;
  double findT2( const double d1, const double d2, const double t1, double t2, const double a_max) const;
  void printStats(const trajectory_msgs::JointTrajectory& trajectory,
                  const std::vector<moveit_msgs::JointLimits>& limits) const;
  void printPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const;
};

}

#endif
