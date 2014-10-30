#include <spiri_motion_primitives_server/spiri_motion_primitives_server.h>
#include <utils/pid.h>

SpiriMotionPrimitivesActionServer::SpiriMotionPrimitivesActionServer(std::string name) :
        as_(nh_, name, boost::bind(&SpiriMotionPrimitivesActionServer::processPrimitiveActionRequest, this, _1), false),
        action_name_(name)
{
    as_.start();
    range_sub_ = nh_.subscribe("range", 1, &SpiriMotionPrimitivesActionServer::range_callback, this);
    
    // TODO(Arnold): Parameterize this
    state_sub_ = nh_.subscribe("/ground_truth/state", 1, &SpiriMotionPrimitivesActionServer::state_callback, this);
}


void SpiriMotionPrimitivesActionServer::range_callback(const sensor_msgs::RangeConstPtr &range)
{
    last_range_ = range;
}

void SpiriMotionPrimitivesActionServer::state_callback(const nav_msgs::OdometryConstPtr &odom)
{
    last_odom_ = odom;
}

void SpiriMotionPrimitivesActionServer::SpiriMotionPrimitivesActionServer::processPrimitiveActionRequest(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    if (goal->action_type == "land")
        this->doLand(goal);
    else if (goal->action_type == "hover")
        this->doHover(goal);
    else if (goal->action_type == "twist")
        this->doTwist(goal);
    else if (goal->action_type == "goto")
        this->doGoTo(goal);
    else
    {
        ROS_INFO("Got bad primitive action request of type: %s", goal->action_type.c_str());
        as_.setAborted();
    }
}

void SpiriMotionPrimitivesActionServer::doLand(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    bool success = false;
    
    ROS_INFO("Executing Land: (speed:%f)", goal->speed);
    
    int counter = 10;
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        if (counter <= 0)
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
        counter -= (int) goal->speed;
        // Here you would update your range measurement
        // And here you would run the PID object
        
        // and put the error in the feedback
        feedback_.error = (double) counter;
        as_.publishFeedback(feedback_);
    }
    
    
    if (success)
    {
        result_.success = success; //true
        ROS_INFO("%s::%s: Succeeded", action_name_.c_str(), goal->action_type.c_str());
        as_.setSucceeded(result_);
    }   
}

double SpiriMotionPrimitivesActionServer::getDistanceToGround()
{
    tf::StampedTransform tf;
    
    try {
        this->getTransformListener()->lookupTransform("range_link", "base_link", 
                           ros::Time(0), tf);
        return last_range->range + tf.getOrigin().z();
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return last_range->range;
    }
}


void SpiriMotionPrimitivesActionServer::doHover(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    bool success = false;
    
    ROS_INFO("Executing Hover: (height:%f, speed:%f)", goal->point.z, goal->speed);
    
    double distance_to_ground;
    PID height_pid(goal->point.z);
    ros::Time last_update_time;
    
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        
        if ( (distance_to_ground < goal->point.z + goal->tolerance) &&
             (distance_to_ground > goal->point.z - goal->tolerance) )
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
        distance_to_ground = getDistanceToGround();
        // Still have to set dt!
        height_pid.update(distance_to_ground, 0.1);
        
        // and put the error in the feedback
        feedback_.error = (double) goal->point.z - counter;
        as_.publishFeedback(feedback_);
    }
    
    
    if (success)
    {
        result_.success = success; //true
        ROS_INFO("%s::%s: Succeeded", action_name_.c_str(), goal->action_type.c_str());
        as_.setSucceeded(result_);
    }  
}

void SpiriMotionPrimitivesActionServer::doTwist(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    
    bool success = false;
    
    ROS_INFO("Executing Twist (x:%f, y:%f, z:%f, yaw:%f) for %f seconds. Ignoring roll and pitch componants", 
        goal->twist.linear.x, goal->twist.linear.y, goal->twist.linear.z, goal->twist.angular.z, goal->time);
        
    feedback_.time_remaining = goal->time;
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        if (feedback_.time_remaining <= 0)
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
        // Have to get dt here
        feedback_.time_remaining -= 0.1;
        as_.publishFeedback(feedback_);
    }
    
    
    if (success)
    {
        result_.success = success; //true
        ROS_INFO("%s::%s: Succeeded", action_name_.c_str(), goal->action_type.c_str());
        as_.setSucceeded(result_);
    }  
}
    

void SpiriMotionPrimitivesActionServer::doGoTo(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    bool success = false;
    
    ROS_INFO("Executing Go To: (x:%f, y:%f, z:%f), (speed:%f, tolerance:%f)",
        goal->point.x, goal->point.y, goal->point.z, goal->speed, goal->tolerance);
        
    // Here you would create a PID object
    
    // Use the feedback message to control the loop
    feedback_.error = 1.0;
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        if (feedback_.error > goal->tolerance)
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
        
        // And here you would run the PID object
        
        // and put the error in the feedback
        as_.publishFeedback(feedback_);
    }
    
    
    if (success)
    {
        result_.success = success; //true
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spiri_motion_primitives");
  ros::NodeHandle nh;
  
  SpiriMotionPrimitivesActionServer spiri_motion_primitives_action_server(ros::this_node::getName());
  
  ros::spin();
  
  return 0;
}


