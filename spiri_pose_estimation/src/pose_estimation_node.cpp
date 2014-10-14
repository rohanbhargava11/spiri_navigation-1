//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <spiri_pose_estimation/pose_estimation_node.h>

#include "hector_pose_estimation/ros/parameters.h"
#include "hector_pose_estimation/system/generic_quaternion_system_model.h"
#include "hector_pose_estimation/measurements/poseupdate.h"
#include "hector_pose_estimation/measurements/baro.h"
#include "hector_pose_estimation/measurements/height.h"
#include "hector_pose_estimation/measurements/magnetic.h"
#include "hector_pose_estimation/measurements/gps.h"

namespace hector_pose_estimation {

SpiriPoseEstimationNode::SpiriPoseEstimationNode(const SystemPtr& system)
  : pose_estimation_(new PoseEstimation(system))
  , private_nh_("~")
  , transform_listener_(0)
{
  if (!system) pose_estimation_->addSystem(System::create(new GenericQuaternionSystemModel));
  
  Magnetic *magnetic = new Magnetic("magnetic");
  magnetic->setStdDev(0.036);
  pose_estimation_->addMeasurement(magnetic);
  
  Height *range = new Height("range");
  range->setStdDev(0.0001);
  pose_estimation_->addMeasurement(range);
  
  Height *pressure = new Height("pressure");
  pressure->setStdDev(1.0);
  pose_estimation_->addMeasurement(pressure);
  
  Height *gpsAltitude = new Height("gpsaltitude");
  gpsAltitude->setStdDev(0.2);
  pose_estimation_->addMeasurement(gpsAltitude);
  
  pose_estimation_->addMeasurement(new GPS("gps"));
  pose_estimation_->addMeasurement(new PoseUpdate("poseupdate"));
}

SpiriPoseEstimationNode::~SpiriPoseEstimationNode()
{
  cleanup();
  delete pose_estimation_;
  delete transform_listener_;
}

bool SpiriPoseEstimationNode::init() {
  // get parameters
  pose_estimation_->parameters().initialize(ParameterRegistryROS(getPrivateNodeHandle()));
  getPrivateNodeHandle().param("publish_covariances", publish_covariances_, false);
  getPrivateNodeHandle().param("publish_world_map_transform", publish_world_other_transform_, false);
  getPrivateNodeHandle().param("map_frame", other_frame_, std::string());

  // search tf_prefix parameter
  tf_prefix_ = tf::getPrefixParam(getPrivateNodeHandle());
  if (!tf_prefix_.empty()) ROS_INFO("Using tf_prefix '%s'", tf_prefix_.c_str());

  // initialize pose estimation
  if (!pose_estimation_->init()) {
    ROS_ERROR("Intitialization of pose estimation failed!");
    return false;
  }

  imu_subscriber_        = getNodeHandle().subscribe("raw_imu", 10, &SpiriPoseEstimationNode::imuCallback, this);
  pressure_subscriber_   = getNodeHandle().subscribe("altimeter", 10, &SpiriPoseEstimationNode::pressureCallback, this);
  range_subscriber_      = getNodeHandle().subscribe("range", 10, &SpiriPoseEstimationNode::rangeCallback, this);
  magnetic_subscriber_   = getNodeHandle().subscribe("magnetic", 10, &SpiriPoseEstimationNode::magneticCallback, this);

  gps_subscriber_.subscribe(getNodeHandle(), "fix", 10);
  gps_velocity_subscriber_.subscribe(getNodeHandle(), "fix_velocity", 10);
  gps_synchronizer_ = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped>(gps_subscriber_, gps_velocity_subscriber_, 10);
  gps_synchronizer_->registerCallback(&SpiriPoseEstimationNode::gpsCallback, this);

  state_publisher_       = getNodeHandle().advertise<nav_msgs::Odometry>("state", 10, false);
  pose_publisher_        = getNodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 10, false);
  velocity_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("velocity", 10, false);
  imu_publisher_         = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 10, false);
  global_publisher_      = getNodeHandle().advertise<sensor_msgs::NavSatFix>("global", 10, false);
  euler_publisher_       = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("euler", 10, false);

  angular_velocity_bias_publisher_    = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("angular_velocity_bias", 10, false);
  linear_acceleration_bias_publisher_ = getNodeHandle().advertise<geometry_msgs::Vector3Stamped>("linear_acceleration_bias", 10, false);
  gps_pose_publisher_                 = getNodeHandle().advertise<geometry_msgs::PoseStamped>("fix/pose", 10, false);

  odomupdate_subscriber_  = getNodeHandle().subscribe("odomupdate", 10, &SpiriPoseEstimationNode::odomupdateCallback, this);
  poseupdate_subscriber_  = getNodeHandle().subscribe("poseupdate", 10, &SpiriPoseEstimationNode::poseupdateCallback, this);
  twistupdate_subscriber_ = getNodeHandle().subscribe("twistupdate", 10, &SpiriPoseEstimationNode::twistupdateCallback, this);
  syscommand_subscriber_  = getNodeHandle().subscribe("syscommand", 10, &SpiriPoseEstimationNode::syscommandCallback, this);

  // publish initial state
  publish();

  return true;
}

void SpiriPoseEstimationNode::reset() {
  pose_estimation_->reset();
}

void SpiriPoseEstimationNode::cleanup() {
  pose_estimation_->cleanup();
  if (gps_synchronizer_) {
    delete gps_synchronizer_;
    gps_synchronizer_ = 0;
  }
}

void SpiriPoseEstimationNode::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  pose_estimation_->setInput(ImuInput(*imu));
  pose_estimation_->update(imu->header.stamp);
  publish();
}

void SpiriPoseEstimationNode::rangeCallback(const sensor_msgs::RangeConstPtr& range) {
  tf::StampedTransform tf;
  try {
    this->getTransformListener()->lookupTransform("range_link", "base_link",  
                           ros::Time(0), tf);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  Height::MeasurementVector update;
  if (range->range < range->max_range - 0.1) {
    update = range->range + tf.getOrigin().z();
    pose_estimation_->getMeasurement("range")->add(Height::Update(update));

  }
  else {
    // This is a dirty hack.
    // Hopefully the range data on the real robot isn't so bad
    // If the pose estimate never leaves the ground this could be to blame
    geometry_msgs::PoseStamped pose_msg;
    pose_estimation_->getPose(pose_msg);
    if ( (pose_msg.pose.position.z + tf.getOrigin().z()) > range->max_range*0.5) {
      // we're closer to the max range - the data is invalid
    }
    else {
      // we're closer to the min range ie should report 0
      update = tf.getOrigin().z();
      pose_estimation_->getMeasurement("range")->add(Height::Update(update));
    }
    //ROS_WARN("range is below minimum height\n");
  }
  
}

void SpiriPoseEstimationNode::pressureCallback(const hector_uav_msgs::AltimeterConstPtr& altimeter) {
  Height::MeasurementVector update;
  update = altimeter->altitude;
  pose_estimation_->getMeasurement("pressure")->add(Height::Update(update));
}


void SpiriPoseEstimationNode::magneticCallback(const geometry_msgs::Vector3StampedConstPtr& magnetic) {
  Magnetic::MeasurementVector update;
  update.x() = magnetic->vector.x;
  update.y() = magnetic->vector.y;
  update.z() = magnetic->vector.z;
  pose_estimation_->getMeasurement("magnetic")->add(Magnetic::Update(update));
}

void SpiriPoseEstimationNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::Vector3StampedConstPtr& gps_velocity) {
  if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return;
  GPS::Update update;
  update.latitude = gps->latitude * M_PI/180.0;
  update.longitude = gps->longitude * M_PI/180.0;
  update.velocity_north =  gps_velocity->vector.x;
  update.velocity_east  = -gps_velocity->vector.y;
  pose_estimation_->getMeasurement("gps")->add(update);
  
  Height::Update heightupdate;
  heightupdate = gps->altitude;
  pose_estimation_->getMeasurement("gpsaltitude")->add(heightupdate);

  if (gps_pose_publisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped gps_pose;
    pose_estimation_->getHeader(gps_pose.header);
    gps_pose.header.seq = gps->header.seq;
    gps_pose.header.stamp = gps->header.stamp;
    GPSModel::MeasurementVector y = boost::static_pointer_cast<GPS>(pose_estimation_->getMeasurement("gps"))->getVector(update, pose_estimation_->state());
    gps_pose.pose.position.x = y(1);
    gps_pose.pose.position.y = y(2);
    gps_pose.pose.position.z = gps->altitude - pose_estimation_->globalReference()->position().altitude;
    double track = atan2(gps_velocity->vector.y, gps_velocity->vector.x);
    gps_pose.pose.orientation.w = cos(track/2);
    gps_pose.pose.orientation.z = sin(track/2);
    gps_pose_publisher_.publish(gps_pose);
  }
}

void SpiriPoseEstimationNode::odomupdateCallback(const nav_msgs::OdometryConstPtr& odom) {
  // Filling in diagonal elements of covariance matrix because the gazebo plugin leaves it as 0
  // since it has no uncertainty, but the EKF treats 0 valued entries as infinity
  
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose = odom->pose;
  pose.header = odom->header;
  pose.pose.covariance[0] = 0.001;
  pose.pose.covariance[7] = 0.001;
  pose.pose.covariance[14] = 0.001;
  pose.pose.covariance[21] = 0.001;
  pose.pose.covariance[28] = 0.001;
  pose.pose.covariance[35] = 0.001;
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(pose));
  
  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.twist = odom->twist;
  twist.header = odom->header;
  twist.twist.covariance[0] = 0.001;
  twist.twist.covariance[7] = 0.001;
  twist.twist.covariance[14] = 0.001;
  twist.twist.covariance[21] = 0.001;
  twist.twist.covariance[28] = 0.001;
  twist.twist.covariance[35] = 0.001; 
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(twist));
}

void SpiriPoseEstimationNode::poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(pose));
}

void SpiriPoseEstimationNode::twistupdateCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist) {
  pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(twist));
}

void SpiriPoseEstimationNode::syscommandCallback(const std_msgs::StringConstPtr& syscommand) {
  if (syscommand->data == "reset") {
    ROS_INFO("Resetting pose_estimation");
    pose_estimation_->reset();
    publish();
  }
}

void SpiriPoseEstimationNode::publish() {
  if (state_publisher_) {
    nav_msgs::Odometry state;
    pose_estimation_->getState(state, publish_covariances_);
    state_publisher_.publish(state);
  }

  if (pose_publisher_) {
    geometry_msgs::PoseStamped pose_msg;
    pose_estimation_->getPose(pose_msg);
    pose_publisher_.publish(pose_msg);
  }

  if (imu_publisher_) {
    sensor_msgs::Imu imu_msg;
    pose_estimation_->getHeader(imu_msg.header);
    pose_estimation_->getOrientation(imu_msg.orientation);
    pose_estimation_->getImuWithBiases(imu_msg.linear_acceleration, imu_msg.angular_velocity);
    imu_publisher_.publish(imu_msg);
  }

  if (velocity_publisher_) {
    geometry_msgs::Vector3Stamped velocity_msg;
    pose_estimation_->getVelocity(velocity_msg);
    velocity_publisher_.publish(velocity_msg);
  }

  if (global_publisher_) {
    sensor_msgs::NavSatFix global_msg;
    pose_estimation_->getGlobalPosition(global_msg);
    global_publisher_.publish(global_msg);
  }

  if (euler_publisher_) {
    geometry_msgs::Vector3Stamped euler_msg;
    pose_estimation_->getHeader(euler_msg.header);
    pose_estimation_->getOrientation(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);
    euler_publisher_.publish(euler_msg);
  }

  if (angular_velocity_bias_publisher_ || linear_acceleration_bias_publisher_) {
    geometry_msgs::Vector3Stamped angular_velocity_msg, linear_acceleration_msg;
    pose_estimation_->getBias(angular_velocity_msg, linear_acceleration_msg);
    if (angular_velocity_bias_publisher_) angular_velocity_bias_publisher_.publish(angular_velocity_msg);
    if (linear_acceleration_bias_publisher_) linear_acceleration_bias_publisher_.publish(linear_acceleration_msg);
  }

  if (getTransformBroadcaster())
  {
    transforms_.clear();

    pose_estimation_->getTransforms(transforms_);

    if (publish_world_other_transform_) {
      tf::StampedTransform world_to_other_transform;
      std::string nav_frame = pose_estimation_->parameters().getAs<std::string>("nav_frame");
      try {
        getTransformListener()->lookupTransform(nav_frame, other_frame_, ros::Time(), world_to_other_transform);
        pose_estimation_->updateWorldToOtherTransform(world_to_other_transform);
        transforms_.push_back(world_to_other_transform);

      } catch (tf::TransformException& e) {
        ROS_WARN("Could not find a transformation from %s to %s to publish the world transformation", nav_frame.c_str(), other_frame_.c_str());
      }
    }

    // resolve tf frames
    for(std::vector<tf::StampedTransform>::iterator t = transforms_.begin(); t != transforms_.end(); t++) {
      t->frame_id_       = tf::resolve(tf_prefix_, t->frame_id_);
      t->child_frame_id_ = tf::resolve(tf_prefix_, t->child_frame_id_);
    }

    getTransformBroadcaster()->sendTransform(transforms_);
  }
}

tf::TransformBroadcaster *SpiriPoseEstimationNode::getTransformBroadcaster() {
  return &transform_broadcaster_;
}

tf::TransformListener *SpiriPoseEstimationNode::getTransformListener() {
  if (!transform_listener_) transform_listener_ = new tf::TransformListener();
  return transform_listener_;
}

} // namespace hector_pose_estimation
