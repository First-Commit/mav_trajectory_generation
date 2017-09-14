#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

class PathGen {
public: 
  PathGen();

private:
  bool buildTrajectory(const Eigen::MatrixXd &waypoints, int derivToMin);
  void setCurrentTime(double time);
  void setPlaySpeed(float speed);
  double dt();
  void advanceTime(float speed);
  Eigen::VectorXd getPosition(float time);
  void publishPose();
  void onPlay(const std_msgs::Float64::ConstPtr& msg);

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  ros::Subscriber play_sub_;

  std::shared_ptr<mav_trajectory_generation::Trajectory> traj_;
  float speed_;
  float current_time_;
  float hz_;
  float start_time_;
  float end_time_;
};