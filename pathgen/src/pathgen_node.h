#include <sstream>
#include "geometry_msgs/Twist.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Trigger.h"
#include "pathgen/PathGenStatus.h"
#include <tf/transform_datatypes.h>
#include "csv.h"
#include <iostream>
#include <fstream>

class PathGen {
 public:
  PathGen(const std::string &path_file, const std::string &out_file);

 private:
  bool buildTrajectory(int derivToMin);
  void setCurrentTime(double time);
  void setPlaySpeed(float speed);
  void advanceTime(float speed);
  Eigen::VectorXd getPosition(double time);
  void publishPose();
  void publishStatus();
  void onPlay(const std_msgs::Float64::ConstPtr& msg);
  void onPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  bool onBuild(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &reply);
  bool addWaypointFront(const Eigen::Vector4d &current_pose,
                                    Eigen::MatrixXd &waypoints);
  geometry_msgs::Vector3 quatToEuler(geometry_msgs::Quaternion &quat);

  void loadPathData(const std::string &path_file, Eigen::MatrixXd &waypoints);
  void printPathData(const Eigen::MatrixXd &waypoints);
  void saveTrajectory(const std::string &out_file);

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber play_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer build_traj_service_;

  std::shared_ptr<mav_trajectory_generation::Trajectory> traj_;
  float speed_;
  double current_time_;
  double hz_;
  double start_time_;
  double end_time_;

  Eigen::Vector4d current_pose_;
  Eigen::MatrixXd waypoints_;
  std::vector<Eigen::Vector4d> path_waypoints_;

  bool path_loaded_;
  bool first_play_;
  unsigned int cnt_;

};