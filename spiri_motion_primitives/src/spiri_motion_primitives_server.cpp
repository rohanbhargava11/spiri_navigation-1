#include <spiri_motion_primitives_server/spiri_motion_primitives_server.h>

SpiriMotionPrimitivesActionServer::SpiriMotionPrimitivesActionServer(std::string name) :
        as_(nh_, name, boost::bind(&SpiriMotionPrimitivesActionServer::processPrimitiveActionRequest, this, _1), false),
        action_name_(name)
{
    as_.start();
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


void SpiriMotionPrimitivesActionServer::doHover(const spiri_motion_primitives::SpiriPrimitiveGoalConstPtr &goal)
{
    bool success = false;
    
    ROS_INFO("Executing Hover: (height:%f, speed:%f)", goal->point.z, goal->speed);
    
    int counter = 0;
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        if (counter > goal->point.z - goal->tolerance and counter < goal->point.z + goal->tolerance)
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
        counter += (int) goal->speed;
        // Here you would update your range measurement
        // And here you would run the PID object
        
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


