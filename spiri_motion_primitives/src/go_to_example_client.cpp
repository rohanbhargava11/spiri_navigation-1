#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <spiri_motion_primitives/SpiriMoveToAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "example_go_to");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction> ac("spiri_motion_primitives", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    spiri_motion_primitives::SpiriMoveToGoal goal;

    
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "base_link";
    goal.pose.pose.position.x = 3.0;
    goal.pose.pose.position.y = 2.0;
    goal.pose.pose.position.z = 1.0;
    
    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    goal.speed = 1.0;
    goal.tolerance = 0.1;

    goal.use_distance_from_ground = false;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ac.cancelGoal();
        ROS_INFO("Action did not finish before the time out.");
    }

    //exit
    return 0;
}
