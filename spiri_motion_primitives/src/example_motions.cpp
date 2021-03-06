#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <spiri_motion_primitives/SpiriMoveToAction.h>

/*
 * Takeoff
 * +X
 * +Y
 * +Z
 * -X
 * -Y
 * +Yaw
 * Go To Global 1, 1, 1
 * Land
 */

bool sendGoal(actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> *ac, spiri_motion_primitives::SpiriMoveToGoal goal)
{
    ac->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
    }
    else
    {
        ac->cancelGoal();
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
}

bool takeoff(actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> *ac, double height)
{
    spiri_motion_primitives::SpiriMoveToGoal goal;
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "world";
    goal.pose.pose.position.x = 0.0;
    goal.pose.pose.position.y = 0.0;
    goal.pose.pose.position.z = height;
    
    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    goal.speed = 1.0;
    goal.tolerance = 0.1;
    goal.use_distance_from_ground = true;
    
    return sendGoal(ac, goal);
}

bool land(actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> *ac)
{
    spiri_motion_primitives::SpiriMoveToGoal goal;
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "world";
    goal.pose.pose.position.x = 0.0;
    goal.pose.pose.position.y = 0.0;
    goal.pose.pose.position.z = 0.0;
    
    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    goal.speed = 2.0;
    goal.tolerance = 0.05;
    goal.use_distance_from_ground = true;
    
    return sendGoal(ac, goal);
}

bool move_relative(actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> *ac, double x, double y, double z, double yaw)
{
    spiri_motion_primitives::SpiriMoveToGoal goal;
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "base_link";
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;
    
    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    
    goal.speed = 1.0;
    goal.tolerance = 0.1;
    goal.use_distance_from_ground = false;
    
    return sendGoal(ac, goal);
}

bool move_world(actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> *ac, double x, double y, double z, double yaw)
{
    spiri_motion_primitives::SpiriMoveToGoal goal;
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "world";
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;
    
    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    
    goal.speed = 1.0;
    goal.tolerance = 0.1;
    goal.use_distance_from_ground = false;
    
    return sendGoal(ac, goal);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "example_motions");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> ac("spiri_motion_primitives", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started.");
    ros::Duration s(0.1);
    
    double z = 2;
    takeoff(&ac, z);
    s.sleep();
    //move_relative(&ac, 0, 0, 0, M_PI);
    //s.sleep();
    
    double R = 3.0;
    int N = 45;
    for (int i = 0; i <= N; i++)
    {
        double theta = i*(2*M_PI/N);
        double x = R*cos(theta);
        double y = R*sin(theta);
        double psi = -M_PI/2 + atan2((R-x), y);
        move_world(&ac, x, y, z, psi);
        //s.sleep();
    }
    
    
    /*for (int i = 0; i < 3; i++)
        {
        move_relative(&ac, 1.0, 0.0, 0.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, 1.0, 0.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, 0.0, 1.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, 0.0, 0.0, 1.0);
        s.sleep();
        move_relative(&ac, -1.0, 0.0, 0.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, -1.0, 0.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, 0.0, -1.0, 0.0);
        s.sleep();
        move_relative(&ac, 0.0, 0.0, 0.0, -1.0);
        s.sleep();
    }*/

    move_world(&ac, 0.0,0.0,1.0,0.0);
    s.sleep();
    land(&ac);
    
    
    
   
    //exit
    return 0;
}
