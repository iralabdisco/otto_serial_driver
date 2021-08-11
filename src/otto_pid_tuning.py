#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rospy.client import load_command_line_node_params
import tf
from otto_serial_driver.msg import otto_ticks, otto_pid_tuning_values

baseline = 0.0
ticks_per_revolution = 0
left_wheel_circ = 0.0
right_wheel_circ = 0.0

pid_pub = rospy.Publisher('/otto_pid_tuning', otto_pid_tuning_values, queue_size=10)
pid_msg = otto_pid_tuning_values()

def cmd_vel_callback(data):
    global baseline
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    pid_msg.left_setpoint = linear_vel - (baseline * angular_vel) / 2
    pid_msg.right_setpoint = linear_vel + (baseline * angular_vel) / 2
    pid_msg.cross_setpoint = pid_msg.left_setpoint - pid_msg.right_setpoint

def otto_ticks_callback(data):
    global ticks_per_revolution, left_wheel_circ, right_wheel_circ

    left_arc = data.left_ticks * left_wheel_circ / ticks_per_revolution
    right_arc = data.right_ticks * right_wheel_circ / ticks_per_revolution
    pid_msg.left_velocity = left_arc/(data.delta_millis/1000)
    pid_msg.right_velocity = right_arc/(data.delta_millis/1000)
    pid_msg.cross_velocity = pid_msg.left_velocity - pid_msg.right_velocity
    pid_pub.publish(pid_msg)

def otto_pid_tuning():
    ## ROS init node
    rospy.init_node('otto_pid_tuning', anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(10)

    ## get coniguration parameters
    global baseline, ticks_per_revolution, left_wheel_circ, right_wheel_circ

    baseline = rospy.get_param("baseline", 0.435)
    ticks_per_revolution = rospy.get_param("ticks_per_revolution", 148000) 
    left_wheel_circ = rospy.get_param("left_wheel_circ", 0.789)
    right_wheel_circ = rospy.get_param("right_wheel_circ", 0.783)

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/otto_ticks', otto_ticks, otto_ticks_callback)
    
    while (not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    otto_pid_tuning()
