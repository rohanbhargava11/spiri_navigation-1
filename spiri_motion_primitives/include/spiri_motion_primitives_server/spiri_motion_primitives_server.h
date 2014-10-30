#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <spiri_motion_primitives/SpiriPrimitiveAction.h>

#ifndef SPIRI_MOTION_PRIMITIVES_SERVER
#define SPIRI_MOTION_PRIMITIVES_SERVER

typedef actionlib::SimpleActionServer<spiri_motion_primitives::SpiriPrimitiveAction> PrimitivesServer;

class SpiriMotionPrimitivesActionServer
{
  public:
    SpiriMotionPrimitivesActionServer(std::string name);
    
    ~SpiriMotionPrimitivesActionServer(void) { }
    
    void processPrimitiveActionRequest(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal);
    
    void doGoTo(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal);
    void doLand(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal);
    void doHover(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal);
    void doTwist(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal);
    
    void range_callback(const sensor_msgs::RangeConstPtr &range);
    void state_callback(const nav_msgs::OdometryConstPtr &odom);
  
  protected:
    ros::NodeHandle nh_;
    void preemptAllServers(void);
    
    PrimitivesServer as_;
    std::string action_name_;
    ros::Subscriber state_sub_;
    ros::Subscriber range_sub_;
    ros::Publisher cmd_vel_pub_;
    
    sensor_msgs::RangeConstPtr last_range_;
    nav_msgs::OdometryConstPtr last_odom_;
    
    tf::Listener tf_listener;

    spiri_motion_primitives::SpiriPrimitiveFeedback feedback_;
    spiri_motion_primitives::SpiriPrimitiveResult result_;
};

#endif //SPIRI_MOTION_PRIMITIVES_SERVER
