#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <otto_serial_driver/otto_ticks.h>

float baseline;
float left_wheel_circ;
float right_wheel_circ;
int ticks_per_revolution;

ros::Publisher odom_pub;

float global_x;
float global_y;
float global_th;

void ticks_callback(const otto_serial_driver::otto_ticks::ConstPtr& msg)
{
  uint32_t left_ticks = msg->left_ticks;
  uint32_t right_ticks = msg->right_ticks;

  float ssx = left_ticks*left_wheel_circ/ticks_per_revolution;
  float sdx = right_ticks*right_wheel_circ/ticks_per_revolution;

  float diff = sdx - ssx;
  
  float delta_x;
  float delta_y;
  float delta_th;

  if (abs(diff) <= 0.0001f) {
    delta_x = ssx;
    delta_y = 0;
    delta_th = 0;
  } else {
      float radius = (baseline / 2) + ((sdx * baseline) / diff);
      float alpha = diff / baseline;
      delta_x = radius * sin(alpha);
      delta_y = radius * cos(alpha) - radius;
      delta_th = alpha;
    }

  global_x = global_x + delta_x* cos(global_th) - delta_y * sin(global_th);
  global_y = global_y + delta_x * sin(global_th) + delta_y * cos(global_th);
  global_th = global_th + global_th;
  
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(global_th);

  //first, we'll publish the transform over tf

  tf::TransformBroadcaster odom_broadcaster;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = global_x;
  odom_trans.transform.translation.y = global_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = global_x;
  odom.pose.pose.position.y = global_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  //publish the message
  odom_pub.publish(odom);

}

int main( int argc, char** argv ){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  n.getParam("/baseline", baseline);
  n.getParam("/left_wheel_circ", left_wheel_circ);
  n.getParam("/right_wheel_circ", right_wheel_circ);
  n.getParam("/ticks_per_revolution", ticks_per_revolution);
  
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber ticks_sub = n.subscribe("otto_ticks", 1000, ticks_callback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Rate r(20.0);

  while(n.ok()){
    ros::spinOnce();  // check for incoming messages
    r.sleep();
  }
}
