#include <attitude_control/attitude_control.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>

AttitudeController::AttitudeController()
{
    nh_.param<double>("spiri_attitude_control/kp", kp_, 0.6);
    nh_.param<double>("spiri_attitude_control/ki", ki_, 0.03);
    nh_.param<double>("spiri_attitude_control/kd", kd_, 0.003);
    nh_.param<double>("spiri_attitude_control/max_roll", max_roll_, M_PI/4);
    nh_.param<double>("spiri_attitude_control/max_pitch", max_pitch_, M_PI/4);
    nh_.param<double>("spiri_attitude_control/spiri_mass", m_, 0.9);
    
    attitude_pub_ = nh_.advertise<spiri_ros_drivers::AttitudeStamped>("/command/attitude", 1);
    
    // Have to make sure the feedback is fresh whenever we get a command
    cmd_vel_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/command/velocity", 10);
    state_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/state", 10);
    cmd_feedback_sync_ = new message_filters::Synchronizer<CmdFeedbackSyncPolicy>(CmdFeedbackSyncPolicy(10),  *cmd_vel_sub_, *state_sub_);
    cmd_feedback_sync_->registerCallback(boost::bind(&AttitudeController::cmdFeedbackCallback, this, _1, _2));        
    
    // TODO(Arnold): experiment with max accumulator values
    y_pid = PID(kp_, ki_, kd_, 100, 10, true);
    x_pid = PID(kp_, ki_, kd_, 100, 10, true);
    z_pid = PID(kp_, ki_, kd_, 100, 10, true);
}


void AttitudeController::cmdFeedbackCallback(const geometry_msgs::TwistStampedConstPtr& cmd_vel, const nav_msgs::OdometryConstPtr& state)
{
    double dt = (ros::Time::now() - last_time_).toSec();
    last_time_ = ros::Time::now();
    
    
    // Get the velocity in the base_link frame
    tf::StampedTransform transform;
    try{
      tf_listener.lookupTransform(state->header.frame_id, "base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    // We only want the rotation part of the transform
    transform.setOrigin(tf::Vector3(0, 0, 0));
    
    //Get the status in a format tf library can use
    tf::Vector3 current_velocity_source_frame;
    tf::vector3MsgToTF(state->twist.twist.linear, current_velocity_source_frame);
    
    // Apply the tf to the status
    tf::Vector3 current_velocity = transform(current_velocity_source_frame);
    
    // Update the PID with the velocity error
    double accel_cmd_x = x_pid.update(cmd_vel->twist.linear.x - current_velocity.getX(), dt);
    double accel_cmd_y = y_pid.update(cmd_vel->twist.linear.y - current_velocity.getY(), dt);
    double accel_cmd_z = z_pid.update(cmd_vel->twist.linear.z - current_velocity.getZ(), dt);
    
    double thrust = m_*accel_cmd_z;

    double yaw_cmd = tf::getYaw(state->pose.pose.orientation) + cmd_vel->twist.linear.z;

    spiri_ros_drivers::AttitudeStamped attitude_cmd;
    attitude_cmd.attitude.thrust = thrust;
    attitude_cmd.attitude.pitch = getAngleFromAcceleration(thrust, accel_cmd_x);
    attitude_cmd.attitude.roll = getAngleFromAcceleration(thrust, accel_cmd_y);
    attitude_cmd.attitude.yaw = yaw_cmd;
    
    attitude_cmd.header.stamp = ros::Time::now();
    clampAttitude(attitude_cmd);
    attitude_pub_.publish(attitude_cmd);
}

double AttitudeController::getThrustFromRPM(int RPM)
{
    double T = 0.00000065920745 * pow(RPM, 2-0.009967366) * RPM + 15.4755244;
    return T;
}

double AttitudeController::getAngleFromAcceleration(double thrust, double acceleration)
{
    double P = asin(m_*acceleration/thrust);
    return P;
}

void AttitudeController::clampAttitude(spiri_ros_drivers::AttitudeStamped& attitude)
{
    // Wrap the angles in [-PI, PI] before clamping
    while (attitude.attitude.roll > M_PI) attitude.attitude.roll -= 2*M_PI;
    while (attitude.attitude.roll < -M_PI) attitude.attitude.roll += 2*M_PI;
    
    while (attitude.attitude.pitch > M_PI) attitude.attitude.pitch -= 2*M_PI;
    while (attitude.attitude.pitch < -M_PI) attitude.attitude.pitch += 2*M_PI;
    
    while (attitude.attitude.yaw > M_PI) attitude.attitude.yaw -= 2*M_PI;
    while (attitude.attitude.yaw < -M_PI) attitude.attitude.yaw += 2*M_PI;
    
    
    
    if (attitude.attitude.roll > max_roll_) attitude.attitude.roll = max_roll_;
    if (attitude.attitude.roll < -max_roll_) attitude.attitude.roll = -max_roll_;
    
    if (attitude.attitude.pitch > max_pitch_) attitude.attitude.pitch = max_pitch_;
    if (attitude.attitude.pitch < -max_pitch_) attitude.attitude.pitch = -max_pitch_;
    
    // No max yaw
}

double AttitudeController::wrapAngle(double angle)
{
    while (angle > M_PI) angle -= M_PI;
    while (angle < -M_PI) angle += M_PI;
    return angle;
}
    
int main(int argc, char** argv)
{
  ros::init(argc, argv, "spiri_attitude_control");
  ros::NodeHandle nh;
  
  AttitudeController attitude_controller();
  
  ros::spin();
  
  return 0;
}


