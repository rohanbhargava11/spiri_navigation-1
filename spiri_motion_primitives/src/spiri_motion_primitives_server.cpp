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
    
    geometry_msgs::PoseStamped pose_goal;
    
    // Transform the goal into the requested frame
    if ( (goal->pose.header.frame_id == "") || goal->pose.header.frame_id == "world")
    {
        pose_goal = goal->pose;
    }
    else
    {
        try {
            tf_listener.waitForTransform("world", goal->pose.header.frame_id, ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("world", goal->pose, pose_goal);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("Aborting MoveTo. %s",ex.what());
            as_.setAborted();
            return;
        }
    }
    
    ROS_INFO("Goal in world frame x:%f, y:%f, z:%f", pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z);
        
    // Pull the Yaw out of the orientation quaternion
    tf::Quaternion temp_q;
    tf::quaternionMsgToTF(pose_goal.pose.orientation, temp_q);
    double yaw_goal = tf::getYaw(temp_q);
    
    // TODO(Arnold): Get these from the parameter server
    double kp = 0.6*goal->speed;
    double ki = 0.0*goal->speed;
    double kd = 0.0*goal->speed;
    double max_err = 100;
    double max_acc = 10;
    
    PID x_pid(pose_goal.pose.position.x, kp, ki, kd, max_err, max_acc);
    PID y_pid(pose_goal.pose.position.y, kp, ki, kd, max_err, max_acc);
    PID z_pid(pose_goal.pose.position.z, kp, ki, kd, max_err, max_acc);
    PID yaw_pid(yaw_goal, kp, ki, kd, max_err, max_acc);

    geometry_msgs::PoseStamped current_pose, current_pose_world_frame;
    geometry_msgs::Twist cmd_vel;
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
        current_pose.header = current_odom_->header;
        current_pose.pose = current_odom_->pose.pose;
        tf_listener.transformPose("world", current_pose, current_pose_world_frame);
        
        tf::quaternionMsgToTF(current_pose_world_frame.pose.orientation, temp_q);
        current_yaw = tf::getYaw(temp_q);
        
        if (goal->use_distance_from_ground)
            current_pose_world_frame.pose.position.z = getDistanceToGround();
        
        
        dt = (last_time - current_pose_world_frame.header.stamp).toSec();
        last_time = current_pose_world_frame.header.stamp;
        
        // Update the PIDs
        cmd_vel.linear.x = x_pid.update(current_pose_world_frame.pose.position.x, dt);
        cmd_vel.linear.y = y_pid.update(current_pose_world_frame.pose.position.y, dt);
        cmd_vel.linear.z = z_pid.update(current_pose_world_frame.pose.position.z, dt);
        cmd_vel.angular.z = yaw_pid.update(current_yaw, dt);
        
        // Publish the new velocity command
        //Just to be sure, explicity set angular x and y to 0
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel_pub_.publish(cmd_vel);
                
        // and put the error in the feedback
        feedback_.position_error.x = x_pid.getLastError();
        feedback_.position_error.y = y_pid.getLastError();
        feedback_.position_error.z = z_pid.getLastError();
        feedback_.yaw_error = yaw_pid.getLastError();
        feedback_.sum_sq_error = sqrt( feedback_.position_error.x*feedback_.position_error.x +
                                       feedback_.position_error.y*feedback_.position_error.y +
                                       feedback_.position_error.z*feedback_.position_error.z +
                                       feedback_.yaw_error*feedback_.yaw_error );
        
        //ROS_INFO("ERROR: X:%f Y:%f, Z:%f, YAW:%F",  feedback_.position_error.x,  feedback_.position_error.y,  feedback_.position_error.z,  feedback_.yaw_error);
        
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


