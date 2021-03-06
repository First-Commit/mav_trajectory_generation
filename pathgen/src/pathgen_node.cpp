#include "pathgen_node.h"

PathGen::PathGen(const std::string &path_file, const std::string &out_file)
    : pose_pub_(n_.advertise<geometry_msgs::Twist>("/fc/cmd/pose", 10)),
      status_pub_(n_.advertise<pathgen::PathGenStatus>("/fc/path/status", 10)),
      play_sub_(n_.subscribe("/fc/path/play", 10, &PathGen::onPlay, this)),
      pose_sub_(n_.subscribe("/pose", 10, &PathGen::onPose, this)), // from snav
      build_traj_service_(n_.advertiseService("/fc/path/build", &PathGen::onBuild, this)),
      speed_(0.0),
      current_time_(0.0),
      hz_(20),
      first_play_(false),
      path_loaded_(false),
      current_pose_(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)),
      cnt_(0) {
  ros::Rate loop_rate(hz_);

  // pre-load waypoints
  loadPathData(path_file, waypoints_);
  printPathData(waypoints_);
  if(!out_file.empty()) {
    saveTrajectory(out_file);
    return;
  }

  ROS_INFO("Running at %0.1f Hz", hz_);

  while (ros::ok()) {
    advanceTime(speed_);
    publishPose();
    if(cnt_ % 20 == 0) {
      publishStatus();
      cnt_ = 1;
    }
    ros::spinOnce();

    loop_rate.sleep();
    cnt_++;
  }
}

void PathGen::saveTrajectory(const std::string &out_file) {
  auto res_traj = buildTrajectory(/*derivToMin=*/1);
  ROS_INFO("Writing trajectory to %s", out_file.c_str());
  Eigen::VectorXd intervals = Eigen::VectorXd::LinSpaced(500, 0, traj_->getMaxTime());
  std::ofstream myfile;
  myfile.open(out_file);
  myfile << "x,y,z,yaw" << "\n";
  for (int i = 0; i < intervals.size(); i++) {
    if(intervals[i] < traj_->getMaxTime()) {
      auto pose = getPosition(intervals[i]);
      ROS_INFO("Writing %0.2f %0.3f %0.3f %0.3f %0.3f", intervals[i], pose[0], pose[1], pose[2], pose[3]);
      myfile << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "\n";
    }
  }
  myfile.close();
  ROS_INFO("Done, wrote %d points", (int)(intervals.size()));
}

void PathGen::loadPathData(const std::string &path_file, Eigen::MatrixXd &waypoints) {
  std::vector<Eigen::Vector4d> path;
  io::CSVReader<4> in(path_file);
  in.read_header(io::ignore_extra_column, "x", "y", "z", "yaw");
  float x,y,z,yaw;
  try {
    while(in.read_row(x, y, z, yaw)){
      path.push_back(Eigen::Vector4d(x,y,z,yaw));
    }
    waypoints = Eigen::MatrixXd(4, path.size());
    for(int i=0; i<path.size(); i++) {
      waypoints.col(i) = path[i];
    }
  }
  catch(const std::exception &e) {
    ROS_WARN("Caught exception reading csv, %s", e.what());
  }
}

void PathGen::printPathData(const Eigen::MatrixXd &waypoints) {
  ROS_INFO("Path waypoints, count: %d", (int)(waypoints.cols()));
  for(int i=0; i<waypoints.cols(); i++) {
    ROS_INFO("pt: %d | x %0.3f y %0.3f z %0.3f yaw %0.3f", i, 
            waypoints(0,i), waypoints(1,i), waypoints(2,i), waypoints(3,i));
  }
}

void PathGen::onPlay(const std_msgs::Float64::ConstPtr &msg) {
  if (!path_loaded_) {
    ROS_INFO("Ignoring play command, path not loaded");
    return;
  }
  auto new_speed = (float)msg->data;
  if (new_speed != speed_) {
    speed_ = new_speed;
    ROS_INFO("Setting new speed: %f", new_speed);
  }
}

void PathGen::onPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  geometry_msgs::PoseStamped pose_stamped = *msg;
  auto pos = pose_stamped.pose.position;
  auto quat = pose_stamped.pose.orientation;
  auto rpy = quatToEuler(quat); // return geometry_msgs::Vector3
  current_pose_ << pos.x, pos.y, pos.z, rpy.z;
}

geometry_msgs::Vector3 PathGen::quatToEuler(geometry_msgs::Quaternion &quat) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  geometry_msgs::Vector3 vec;
  vec.x = roll;
  vec.y = pitch;
  vec.z = yaw;
  return vec;
}

bool PathGen::onBuild(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &reply) {
  ROS_INFO("Got new build command!");
  auto res_wps = addWaypointFront(current_pose_, waypoints_);
  // if our waypoints get built, dump out
  if (!res_wps) {
    reply.success = false;
    ROS_ERROR("Failed to build trajectory");
    return true;
  }
  auto res_traj = buildTrajectory(/*derivToMin=*/1);
  // if our traj fails to build, dump out
  if (!res_traj) {
    reply.success = false;
    ROS_ERROR("Failed to build trajectory");
    return true;
  }
  // we're all good
  reply.success = true;
  return true;
}

bool PathGen::addWaypointFront(const Eigen::Vector4d &current_pose, Eigen::MatrixXd &waypoints) {
  if (waypoints_.size() == 0) {
    ROS_ERROR("No waypoints have been specified!");
    return false;
  }
  if (current_pose.x() == 0.0 && current_pose.y() == 0.0) {
    ROS_ERROR("Current position is invalid, can't append to waypoints");
    return false;
  }

  ROS_INFO("Appending waypoint to front of list: %0.3f %0.3f %0.3f %0.3f", current_pose.x(),
           current_pose.y(), current_pose.z(), current_pose(3));

  // increment columns by 1 since we'll add one to the front
  Eigen::MatrixXd wps(waypoints.rows(), waypoints.cols() + 1);
  wps.col(0) =
      (Eigen::VectorXd(4) << current_pose.x(), current_pose.y(), current_pose.z(), current_pose(3))
          .finished();
  for (int i = 0; i < waypoints.cols(); i++) {
    wps.col(i + 1) = waypoints.col(i);
  }
  // overwrite the previous waypoints
  waypoints = wps;
  return true;
}

void PathGen::publishPose() {
  if(!path_loaded_) return;
  auto pose = getPosition(current_time_);
  geometry_msgs::Twist msg;
  msg.linear.x = pose.x();
  msg.linear.y = pose.y();
  msg.linear.z = pose.z();
  msg.angular.z = pose(3);
  pose_pub_.publish(msg);
  ROS_INFO("Time: %f | Pose: x %f y %f z %f yaw %f", current_time_, pose.x(), pose.y(), pose.z(),
           pose(3));
}

void PathGen::publishStatus() {
  pathgen::PathGenStatus msg;
  msg.starttime = start_time_;
  msg.endtime = end_time_;
  msg.currenttime = current_time_;
  msg.playspeed = speed_;
  msg.pathloaded = path_loaded_;
  status_pub_.publish(msg);
  ROS_INFO("Status | cur: %0.3f, end: %0.3f, play: %0.1f, loaded: %d", current_time_, end_time_,
            speed_, path_loaded_);
}

Eigen::VectorXd PathGen::getPosition(double time) {
  if(time > end_time_) {
    time = end_time_;
  }
  return traj_->evaluate(time); 
}

void PathGen::advanceTime(float speed) {
  if (speed == 0) {
    setCurrentTime(current_time_);
  } else {
    setCurrentTime(current_time_ + ((1 / hz_) * speed));
  }
}

void PathGen::setPlaySpeed(float speed) { speed_ = speed; }

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

bool PathGen::buildTrajectory(int derivToMin) {
  if (waypoints_.size() == 0) {
    ROS_ERROR("Can't build trajectory, waypoints are invalid");
    return false;
  }

  // setup the optimization problem
  static constexpr int kDimension = 4;          // x y z yaw
  static constexpr int kPolynomialDegree = 10;  // with 10 we can minimize up to snap (4th deriv)

  // Solve and get trajectory
  mav_trajectory_generation::PolynomialOptimization<kPolynomialDegree> opt(kDimension);
  opt.setupFromWaypoints(waypoints_, derivToMin);
  bool res = opt.solveLinear();
  traj_.reset(new mav_trajectory_generation::Trajectory());
  opt.getTrajectory(&(*traj_));

  start_time_ = 0.0;
  end_time_ = traj_->getMaxTime();
  path_loaded_ = true;

  if (traj_) {
    ROS_INFO("Trajectory built and loaded with %d points", (int)(waypoints_.cols()));
    return true;
  } else
    return false;
}