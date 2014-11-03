#include <spiri_motion_primitives_server/spiri_motion_primitives_server.h>
#include <tf/transform_listener.h>
#include <utils/pid.h>

SpiriMotionPrimitivesActionServer::SpiriMotionPrimitivesActionServer(std::string name) :
        as_(nh_, name, boost::bind(&SpiriMotionPrimitivesActionServer::doMoveTo, this, _1), false),
        action_name_(name)
{
    as_.start();
    range_sub_ = nh_.subscribe("range", 1, &SpiriMotionPrimitivesActionServer::range_callback, this);
    // TODO(Arnold): Parameterize this
    state_sub_ = nh_.subscribe("/ground_truth/state", 1, &SpiriMotionPrimitivesActionServer::state_callback, this);
    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    nh_.param<double>("spiri_motion_primitives/kp", kp_, 0.6);
    nh_.param<double>("spiri_motion_primitives/kp", ki_, 0.03);
    nh_.param<double>("spiri_motion_primitives/kp", kd_, 0.003);
}


void SpiriMotionPrimitivesActionServer::range_callback(const sensor_msgs::RangeConstPtr &range)
{
    current_range_ = range;
    
}

void SpiriMotionPrimitivesActionServer::state_callback(const nav_msgs::OdometryConstPtr &odom)
{
    current_odom_ = odom;
}

double SpiriMotionPrimitivesActionServer::getDistanceToGround()
{
    tf::StampedTransform tf;
    
    try {
        tf_listener.lookupTransform("range_link", "base_link", 
                           ros::Time(0), tf);
        if (current_range_->range >= current_range_->max_range - 0.05) {
            return tf.getOrigin().z();
        }
        else
            return current_range_->range + tf.getOrigin().z();
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return current_range_->range;
    }
}

void SpiriMotionPrimitivesActionServer::doMoveTo(const spiri_motion_primitives::SpiriMoveToGoalConstPtr &goal)
{
    bool success = false;
    
    ROS_INFO("Executing Move To: frame:%s, x:%f, y:%f, z:%f; with speed:%f, tolerance:%f, distance_from_ground:%d",
            goal->pose.header.frame_id.c_str(), goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z,
            goal->speed, goal->tolerance, goal->use_distance_from_ground );
    
    geometry_msgs::PoseStamped goal_world_frame;
    
    // Transform the from the requested frame into the world frame
    if ( (goal->pose.header.frame_id == "") || goal->pose.header.frame_id == "world")
    {
        goal_world_frame = goal->pose;
    }
    else
    {
        try {
            tf_listener.waitForTransform(goal->pose.header.frame_id, "world", ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("world", ros::Time(0), goal->pose, goal->pose.header.frame_id, goal_world_frame);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("Aborting MoveTo. %s",ex.what());
            as_.setAborted();
            return;
        }
    }
    
    //if (goal->use_distance_from_ground)
    //    goal_world_frame.pose.position.z += getDistanceToGround();
    
    ROS_INFO("Goal in world frame x:%f, y:%f, z:%f", goal_world_frame.pose.position.x, goal_world_frame.pose.position.y, goal_world_frame.pose.position.z);
        
    
    double kp = kp_*goal->speed;
    double ki = ki_*goal->speed;
    double kd = kd_*goal->speed;
    double max_err = 100;
    double max_acc = 10;
    
    PID x_pid(kp, ki, kd, max_err, max_acc, false);
    PID y_pid(kp, ki, kd, max_err, max_acc, false);
    PID z_pid(kp, ki, kd, max_err, max_acc, false);
    PID yaw_pid(kp, ki, kd, max_err, max_acc, true);

    geometry_msgs::PoseStamped goal_current_baselink_frame;
    geometry_msgs::Twist cmd_vel;
    tf::Quaternion temp_q;
    double current_yaw;
    double dt;
    
    // Initializing this at time zero will make the derivative term of the PID go to zero on the first iteration
    ros::Time last_time(0.0);
    feedback_.sum_sq_error = -1; // This will only ever be < 0 on the first iteration
    
    // TODO (Arnold): Figure out if this is necessary
    ros::Rate r(30.0);
    
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            // If the client has requested preempt, stop
            as_.setPreempted();
            success = false;
            break;
        }
        
        if (feedback_.sum_sq_error < goal->tolerance and feedback_.sum_sq_error > 0)
        {
            // In this case we've finished!
            success = true;
            break;
        }
        
    
        // Update the state
        tf_listener.waitForTransform("world", "base_link", ros::Time::now(), ros::Duration(1.0));
        tf_listener.transformPose("base_link", ros::Time(0), goal_world_frame, "world", goal_current_baselink_frame);
        
        if (goal->use_distance_from_ground)
            goal_current_baselink_frame.pose.position.z = goal_world_frame.pose.position.z - getDistanceToGround();
       
        
        tf::quaternionMsgToTF(goal_current_baselink_frame.pose.orientation, temp_q);
        current_yaw = tf::getYaw(temp_q);
                
        dt = (ros::Time::now() - last_time).toSec();
        last_time = current_odom_->header.stamp;
        if (dt < 1e-6) dt = 1e-6; // avoid divide by zero. TODO (Arnold): Remove this hack
    
        
        // Update the PIDs. The position of the goal in the base_link frame is the same as the position error
        cmd_vel.linear.x = x_pid.update(goal_current_baselink_frame.pose.position.x, dt);
        cmd_vel.linear.y = y_pid.update(goal_current_baselink_frame.pose.position.y, dt);
        cmd_vel.linear.z = z_pid.update(goal_current_baselink_frame.pose.position.z, dt);
        cmd_vel.angular.z = yaw_pid.update(current_yaw, dt);
        
        // Publish the new velocity command
        //Just to be sure, explicity set angular x and y to 0
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel_pub_.publish(cmd_vel);
                
        // and put the error in the feedback
        // don't really need to do this this way any more
        feedback_.position_error.x = x_pid.getLastError();
        feedback_.position_error.y = y_pid.getLastError();
        feedback_.position_error.z = z_pid.getLastError();
        feedback_.yaw_error = yaw_pid.getLastError();
        
        feedback_.sum_sq_error = sqrt( feedback_.position_error.x*feedback_.position_error.x +
                                       feedback_.position_error.y*feedback_.position_error.y +
                                       feedback_.position_error.z*feedback_.position_error.z +
                                       feedback_.yaw_error*feedback_.yaw_error );
        
        ROS_INFO("dt: %f: ERROR: X:%f Y:%f, Z:%f, YAW:%F",  dt, feedback_.position_error.x,  feedback_.position_error.y,  feedback_.position_error.z,  feedback_.yaw_error);
        
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    
    // Whether the goal is being aborted or we made it, either way we want to stop moving
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
    
    if (success)
    {
        // Put the result back in the frame the goal was sent to us in
        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.header = current_odom_->header;
        tmp_pose.pose = current_odom_->pose.pose;
        tf_listener.waitForTransform(tmp_pose.header.frame_id, goal->pose.header.frame_id, ros::Time::now(), ros::Duration(1.0));
        tf_listener.transformPose(goal->pose.header.frame_id, tmp_pose, result_.pose);
        
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


