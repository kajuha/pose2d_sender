#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <iostream>
#include <queue>

#include <boost/thread.hpp>

using namespace std;

#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D pose2d;
double x_, y_, theta_;

void getPose2d() {
  ros::Rate r(1000);

  while (ros::ok())
  {
    printf("x y theta : ");
    
    scanf("%lf %lf %lf", &x_, &y_, &theta_);

    // ros::spinOnce();

    r.sleep();
  }
}

//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pose2d_sender");
  ros::NodeHandle nh("~");

  string node_name;

  ros::param::get("~node_name", node_name);

  boost::thread threadGetPose2d(getPose2d);

  ros::Publisher pose2d_pub = nh.advertise<geometry_msgs::Pose2D>("pose2d_command", 100);

  ros::Rate r(1000);

#define STEP_TIME 1.0
  double time_cur = ros::Time::now().toSec();
  double time_pre = time_cur;
  double time_diff;

  while (ros::ok())
  {
    time_cur = ros::Time::now().toSec();
    time_diff = time_cur - time_pre;
    if ( time_diff > STEP_TIME ) {
        time_pre = time_cur;
    }

    pose2d.x = x_;
    pose2d.y = y_;
    pose2d.theta = theta_;
    pose2d_pub.publish(pose2d);

    // ros::spinOnce();

    r.sleep();
  }

  // threadGetPose2d.join();

  return 0;
}
