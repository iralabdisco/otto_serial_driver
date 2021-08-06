#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <otto_serial_driver/otto_ticks.h>

float baseline;
float left_wheel_circ;
float right_wheel_circ;
uint32_t ticks_per_revolution;

int main( int argc, char** argv ){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate r(1.0);

  while(n.ok()){

    ros::spinOnce();  // check for incoming messages
    r.sleep();
  }
}
