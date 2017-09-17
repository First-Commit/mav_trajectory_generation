#include "pathgen_node.h"

PathGen::PathGen() : speed_(0.0), 
  current_time_(0.0), hz_(10), 
  pose_pub_(n_.advertise<geometry_msgs::Twist>("/fc/cmd/pose", 10)),
  play_sub_(n_.subscribe("/fc/path/play", 10, &PathGen::onPlay, this)) {
  
  ros::Rate loop_rate(hz_);

  Eigen::MatrixXd waypoints(4, 5); // each col is x,y,z,yaw
  waypoints <<  0,1,1,0,0,
                0,0,1,1,0,
                0,0,0,0,0,
                0,0,0,0,0;
  buildTrajectory(waypoints, 1);

  while (ros::ok()) {
    
    speed_ = 0.5;
    advanceTime(speed_);
    publishPose();

    ros::spinOnce();

    loop_rate.sleep();
  }
}

void PathGen::onPlay(const std_msgs::Float64::ConstPtr& msg) {
  auto new_speed = (float)msg->data;
  if(new_speed != speed_) {
    speed_ = new_speed;
    ROS_INFO("Setting new speed: %f", new_speed);
  }
}

void PathGen::publishPose() {
  auto pose = getPosition(current_time_);
  geometry_msgs::Twist msg;
  msg.linear.x = pose.x();
  msg.linear.y = pose.y();
  msg.linear.z = pose.z();   
  msg.angular.z = pose(3); 
  pose_pub_.publish(msg);
  ROS_INFO("Time: %f | Pose: x %f y %f z %f yaw %f", current_time_, pose.x(), pose.y(), pose.z(), pose(3));
}

Eigen::VectorXd PathGen::getPosition(float time) {
  return traj_->evaluate(time);
}

void PathGen::advanceTime(float speed) {
  if (speed == 0) {
    setCurrentTime(current_time_);
  } else {
    setCurrentTime(current_time_ + ((1/hz_) * speed));
  }
}

void PathGen::setPlaySpeed(float speed) {
  speed_ = speed;
}

void PathGen::setCurrentTime(double time) {
  // clip timer at start and end
  if (time > end_time_) {
    time = end_time_;
    setPlaySpeed(0.0);
  } else if (time < start_time_) {
    time = start_time_;
    setPlaySpeed(0.0);
  }

  if (time != current_time_) {
    current_time_ = time;
  }
}

bool PathGen::buildTrajectory(const Eigen::MatrixXd &waypoints, int derivToMin) {
  // setup the optimization problem
  static constexpr int kDimension = 4; // x y z yaw
  static constexpr int kPolynomialDegree = 10; // with 10 we can minimize up to snap (4th deriv)

  // Solve and get trajectory
  mav_trajectory_generation::PolynomialOptimization<kPolynomialDegree> opt(kDimension);
  opt.setupFromWaypoints(waypoints, derivToMin);
  bool res = opt.solveLinear();
  traj_.reset(new mav_trajectory_generation::Trajectory());
  opt.getTrajectory(&(*traj_));

  start_time_ = 0.0;
  end_time_ = traj_->getMaxTime();
}