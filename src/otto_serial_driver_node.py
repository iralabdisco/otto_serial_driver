#!/usr/bin/env python3
import time

import serial
from serial import SerialException

import struct
from crccheck.crc import Crc32Mpeg2

from math import sin, cos

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from otto_serial_driver.msg import otto_ticks
import numpy as np

ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.2,
        rtscts=False,
        exclusive=None)

linear_vel = 0
angular_vel = 0

def callback(data):
    global linear_vel
    global angular_vel
    linear_vel = data.linear.x
    angular_vel = data.angular.z

def otto_serial_driver():

    ## ROS init node
    rospy.init_node('serial_control', anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(10)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    ticks_pub = rospy.Publisher('otto_ticks', otto_ticks, queue_size=10)
    ticks_msg = otto_ticks()

    ## Serial port opening 
    serial_port = rospy.get_param("serial_port", "/dev/ttyUSB0")
    #dtr is connected to RST, on opening dtr is high by default so it resets the st board
    #after opening the serial port we set it low so the board can boot
    ser.dtr = 0 
    while(ser.is_open == False and not rospy.is_shutdown()):
        try:
            ser.port = serial_port
            ser.open()
        except SerialException:
            rospy.logerr('Couldn\'t open ' + serial_port)
            time.sleep(2)
    rospy.loginfo(serial_port + ' opened')
    reset_st()
    ser.reset_output_buffer()
    ser.reset_input_buffer()
    rate.sleep()
    
    ## init cmd_vel
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0

    ## init odometry
    global_x = 0
    global_y = 0
    global_th = 0

    ## get coniguration parameters
    ticks_per_revolution = rospy.get_param("ticks_per_revolution", 148000)
    baseline = rospy.get_param("baseline", 0.435)

    left_wheel_circ = rospy.get_param("left_wheel_circ", 0.789)
    right_wheel_circ = rospy.get_param("right_wheel_circ", 0.783)

    kp_left = rospy.get_param("kp_left", 180)
    ki_left = rospy.get_param("ki_left", 200)
    kd_left = rospy.get_param("kd_left", 0)

    kp_right = rospy.get_param("kp_right", 185)
    ki_right = rospy.get_param("ki_right", 195)
    kd_right = rospy.get_param("kd_right", 0)

    kp_cross = rospy.get_param("kp_cross", 0)
    ki_cross = rospy.get_param("ki_cross", 0)
    kd_cross = rospy.get_param("kd_cross", 0)

    ## send config to st
    config_msg = struct.pack('<Lffffffffffff', ticks_per_revolution,
                                baseline,
                                left_wheel_circ, right_wheel_circ,
                                kp_left, ki_left, kd_left, 
                                kp_right, ki_right, kd_right,
                                kp_cross, ki_cross, kd_cross)
    ser.write(config_msg)
    crc_tx = Crc32Mpeg2.calc(config_msg)
    # convert crc to bytes and send
    crc_tx_pack = struct.pack('<L', crc_tx)
    ser.write(crc_tx_pack)
    rate.sleep()
    rate.sleep()

    while (not rospy.is_shutdown()):

        ## Transmit
        # convert to bytes and send
        vel_msg = struct.pack('<ff', linear_vel, angular_vel)
        ser.write(vel_msg)
        # calculate crc on byte buffer
        crc_tx = Crc32Mpeg2.calc(vel_msg)
        # convert crc to bytes and send
        crc_tx_pack = struct.pack('<L', crc_tx)
        ser.write(crc_tx_pack)
        
        rospy.logdebug("Sent cmd_vel: Linear %f, Angular %f", linear_vel, angular_vel)
        ## Receive
        # read from serial
        buffer_rx = ser.read(12) #size of status_msg - 4
        crc_rx = ser.read(4) #crc is 32 bits
        status_msg = struct.unpack('=HHll', buffer_rx)
        crc_rx = struct.unpack('=L', crc_rx)[0]
        # calculate crc and check if ok
        crc_rx_calc = Crc32Mpeg2.calc(buffer_rx)
        if (crc_rx_calc != crc_rx):
            rospy.logerr("Error on rx (incorrect CRC)")
        if (status_msg[0] != 1):
            rospy.logerr("Otto error!")
            if (status_msg[0] == 2):
                rospy.logerr("Error on receiving config! Shutting down")
                rospy.signal_shutdown
            elif (status_msg[0] == 3):
                rospy.logerr("Error on receiving vel_msg!")
            elif (status_msg[0] == 4):
                rospy.logerr("H-Bridge fault!, Shutting down")
                rospy.signal_shutdown
            else:
                rospy.logerr("Unknown otto status! Shutting down")
                rospy.signal_shutdown
        else:
            ## publish ticks
            ticks_msg.delta_millis = status_msg[1]
            ticks_msg.left_ticks = status_msg[2]
            ticks_msg.right_ticks = status_msg[3]
            ticks_msg.timestamp = rospy.get_rostime()
            ticks_pub.publish(ticks_msg)

            ## calc odometry values
            current_time = rospy.Time.now()
            left_arc = ticks_msg.left_ticks * left_wheel_circ / ticks_per_revolution
            right_arc = ticks_msg.right_ticks * right_wheel_circ / ticks_per_revolution

            diff =  right_arc - left_arc

            if abs(diff) < 0.001:
                delta_x = left_arc
                delta_y = 0
                delta_th = 0
            else:
                radius = baseline/2 * (right_arc + left_arc) / diff
                delta_th = diff/baseline
                delta_x = radius*sin(delta_th)
                delta_y = radius - radius*cos(delta_th)
            
            global_x = global_x + delta_x* cos(global_th) - delta_y * sin(global_th)
            global_y = global_y + delta_x * sin(global_th) + delta_y * cos(global_th)
            global_th = global_th + delta_th

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, global_th)
            left_vel = left_arc/(ticks_msg.delta_millis/1000)
            right_vel = right_arc/(ticks_msg.delta_millis/1000)
            lin_vel = (left_vel + right_vel)/2
            ang_vel = right_vel - left_vel / baseline

            ## broadcast tf
            odom_broadcaster.sendTransform((global_x, global_y, 0.), odom_quat, current_time, "base_link", "odom")

            ## publish odometry
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(global_x, global_y, 0.), Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(lin_vel, 0, 0), Vector3(0, 0, ang_vel))
            odom_to_publish.pose.covariance = (np.eye(6)*0.02).flatten()
            odom_to_publish.twist.covariance = (np.eye(6)*0.01).flatten()
            odom_pub.publish(odom)

        rospy.logdebug("Received otto status:")
        rospy.logdebug("Status %d, Delta_millis %d, Left_ticks %d, Right_ticks %d", 
                        status_msg[0], status_msg[1], status_msg[2], status_msg[3])
                        
        rate.sleep()

def reset_st():
    ser.dtr = 1
    time.sleep(1)
    ser.dtr = 0

rospy.on_shutdown(reset_st)

if __name__ == '__main__':
    otto_serial_driver()
