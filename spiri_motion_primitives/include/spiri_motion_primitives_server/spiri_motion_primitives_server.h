#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>

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
  
  protected:
    ros::NodeHandle nh_;
    void preemptAllServers(void);
    
    PrimitivesServer as_;
    std::string action_name_;

    spiri_motion_primitives::SpiriPrimitiveFeedback feedback_;
    spiri_motion_primitives::SpiriPrimitiveResult result_;
};

#endif //SPIRI_MOTION_PRIMITIVES_SERVER
