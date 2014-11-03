#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <spiri_motion_primitives/SpiriMoveToAction.h>

#ifndef SPIRI_MOTION_PRIMITIVES_SERVER
#define SPIRI_MOTION_PRIMITIVES_SERVER

typedef actionlib::SimpleActionServer<spiri_motion_primitives::SpiriMoveToAction> MoveToServer;

class SpiriMotionPrimitivesActionServer
{
  public:
    SpiriMotionPrimitivesActionServer(std::string name);
    
    ~SpiriMotionPrimitivesActionServer(void) { }
        
    void doMoveTo(const spiri_motion_primitives::SpiriMoveToGoalConstPtr &goal);
    
    void range_callback(const sensor_msgs::RangeConstPtr &range);
    void state_callback(const nav_msgs::OdometryConstPtr &odom);
  
  protected:
    ros::NodeHandle nh_;
    double getDistanceToGround();
    
    MoveToServer as_;
    std::string action_name_;
    ros::Subscriber state_sub_;
    ros::Subscriber range_sub_;
    ros::Publisher cmd_vel_pub_;
    double kp_, ki_, kd_;
    
    sensor_msgs::RangeConstPtr current_range_;
    nav_msgs::OdometryConstPtr current_odom_;
    
    tf::TransformListener tf_listener;

    spiri_motion_primitives::SpiriMoveToFeedback feedback_;
    spiri_motion_primitives::SpiriMoveToResult result_;
};

#endif //SPIRI_MOTION_PRIMITIVES_SERVER
