#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "ctime"
#include "math.h"
#include "sys/time.h"
#include "nav_msgs/Odometry.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/serialization.h>
#include <position_controller/pid.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

using boost::asio::ip::udp;



boost::asio::io_service io_service;
udp::socket s(io_service,udp::endpoint(udp::v4(),0));
//udp::socket s_mocap(io_service,udp::endpoint(udp::v4(),9050));
udp::endpoint local_endpoint=boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.1.1"),5005);

ros::SerializedMessage data;
double x_pose_pid,y_pose_pid,z_pose_pid,y_pose_pid_optical_flow;
double change_rpm;
double kp=2.0;
double kd=2.0;
double ki=0.0;
double max_err=100.0;
double max_acc=3.0;
double roll,pitch,yaw;
const float KF_spiri = 0.000000054;
double x_position,y_position,y_position_optical_flow,x_position_optflow,y_position_optflow=0.0;
double dt_optical_flow,dt_artag=0.0;
double dt;
PID x_pid(kp, ki, kd, max_err, max_acc, false);
PID y_pid(2.0,ki,2.0,max_err,max_acc,false);
PID y_pid_optical_flow(kp,ki,kd,max_err,max_acc,false);
PID z_pid(10,0.0,4,100,3,false);
ros::Time last_time(1e-6);
char mocap_data [100];
geometry_msgs::Point attitude;
double send_data_spiri[4];
void handle_receive_from(const boost::system::error_code& error,size_t bytes_recvd)
{
    if(!error && bytes_recvd > 0)
    {
        std::cout<<"didn't recive anything"<<"\n";
    }
}
std::vector<std::string> strs;


double yaw_shortest_path(double error_yaw)
{
    error_yaw=error_yaw/(2*3.14);
    int full_rotations = 0;
    float y=0.5;
    if (error_yaw>=0)
    {
        y=0.5;
        full_rotations=std::floor(error_yaw+y);

    }
    else
    {
        y=-0.5;

        full_rotations=std::ceil(error_yaw+y);
    }
    return 2*3.14*(error_yaw-float(full_rotations));
}
double yaw_desired;
void mocap_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    dt = (ros::Time::now() - last_time).toSec();
    last_time=ros::Time::now(); //can't use header stamp as the message is not pose stamped
    ROS_INFO("time elapsed %f",dt);
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation,q);
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    if(yaw<0)
    {
      yaw += 2*3.14;
    }
    
    yaw_desired=yaw_shortest_path(0-yaw);
    yaw_desired=yaw_desired*(180/3.14);
    //yaw_desired=yaw*(180/3.14);
    send_data_spiri[2]=yaw_desired;
    ROS_INFO("yaw in degress %f",yaw_desired);
    x_pose_pid=x_pid.update((-1)*msg->position.y,dt);
    //y_pose_pid=y_pid.update(msg->position.x,dt);
    y_pose_pid=y_pid.update(x_position,dt_artag);
    z_pose_pid=z_pid.update(1.0-msg->position.z,dt);
    
    ROS_INFO("z pid value are %f",z_pose_pid);
    // Sign of axes are reversed
    attitude.y=(1/9.8)*(-x_pose_pid*sin(yaw)+y_pose_pid*cos(yaw))*(180/3.14);
    attitude.x=(1/9.8)*(x_pose_pid*cos(yaw)+y_pose_pid*sin(yaw))*(180/3.14);
    change_rpm=(int)(0.760/(8*KF_spiri*4650))*z_pose_pid;
    ROS_INFO("change rpm %f",change_rpm);
    send_data_spiri[0]=attitude.y;
    send_data_spiri[1]=attitude.x;
    send_data_spiri[3]=4650+(change_rpm);
    
    if (send_data_spiri[0]>25)
    {
      send_data_spiri[0]=25;
    }
    else if(send_data_spiri[0]<-25)
    {
      send_data_spiri[0]=-25;
    }
    
    if (send_data_spiri[1]>25)
    {
      send_data_spiri[1]=25;
    }
    else if(send_data_spiri[1]<-25)
    {
      send_data_spiri[1]=-25;
    }
    if (send_data_spiri[3]>5500)
    {
      send_data_spiri[3]=5500;
    }
    if( send_data_spiri[3]<4000)
    {
      send_data_spiri[3]=4000;
    }
    //send_data_spiri[3]=5000;
    ROS_INFO("x %f and y %f and z %f",(-1)*msg->position.y,x_position,1.0-msg->position.z);
    //send_data_spiri[0]=0.0
    //ROS_INFO("change in RPM %f",change_rpm);
    ROS_INFO("Pitch %f and Roll %f and RPM %f",send_data_spiri[1],send_data_spiri[0],send_data_spiri[3]);
    
    s.send_to(boost::asio::buffer(send_data_spiri),local_endpoint);
    
}



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of tfhe message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  //ros::Subscriber sub = n.subscribe("odom", 1, chatterCallback,ros::TransportHints().unreliable());
  //ros::Timer timer = n.createTimer(ros::Duration(0.05),timercallback);
  //ros::Subscriber sub_artag=n.subscribe("ar_pose_marker",1,ar_tag_callback,ros::TransportHints().unreliable());
  //ros::Subscriber sub_artag=n.subscribe("ar_pose_marker",1,ar_tag_callback_multiple,ros::TransportHints().unreliable());
  ros::Subscriber sub_mocap=n.subscribe("/Robot_1/pose",1,mocap_callback,ros::TransportHints().udp());
  //message_filters::Subscriber<sensor_msgs::Imu> sub_imu(n,"raw_imu",1);
  //message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> sub_optflow(n,"optflow",1);
  //ROS_INFO("setting the callback");
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,geometry_msgs::TwistWithCovarianceStamped> syncpolicy;
  //message_filters::Synchronizer<syncpolicy> sync(syncpolicy(10),sub_imu,sub_optflow);
  //sync.registerCallback(boost::bind(&optflow,_1,_2));

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}
