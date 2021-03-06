#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <utils/pid.h>

#include <spiri_ros_drivers/AttitudeStamped.h>


#ifndef ATTITUDE_CONTROLLER
#define ATTITUDE_CONTROLLER

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, 
                                                        nav_msgs::Odometry,
                                                        sensor_msgs::JointState> CmdFeedbackSyncPolicy;

class AttitudeController
{
  public:
    AttitudeController(void);
  
  protected:
    PID x_pid, y_pid, z_pid;
    double m_;
  
    ros::NodeHandle nh_;
    void cmdFeedbackCallback(const geometry_msgs::TwistStampedConstPtr& cmd_vel,
                             const nav_msgs::OdometryConstPtr& state,
                             const sensor_msgs::JointStateConstPtr& motor_state);

    double getThrustFromRPM(int RPM);
    int getRPMFromThrust(double thrust);
    double getAngleFromAcceleration(double thrust, double acceleration);
    void clampAttitude(spiri_ros_drivers::AttitudeStamped& attitude);

    
    message_filters::Subscriber<geometry_msgs::TwistStamped> *cmd_vel_sub_;      
    message_filters::Subscriber<nav_msgs::Odometry> *state_sub_;      
    message_filters::Subscriber<sensor_msgs::JointState> *motor_state_sub_;      
    message_filters::Synchronizer<CmdFeedbackSyncPolicy> *cmd_feedback_sync_;

    ros::Publisher attitude_pub_;
    double kp_, ki_, kd_;
    double max_pitch_, max_roll_;
    const static double A = 0.0000065920745;
    const static double B = -0.009967366;
    const static double C = 15.4755244;

    ros::Time last_time_;
    
    tf::TransformListener tf_listener;
};

#endif //ATTITUDE_CONTROLLER
