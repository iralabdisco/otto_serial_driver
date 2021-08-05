#!/usr/bin/env python3
import time

import serial
from serial import SerialException

import struct
from crccheck.crc import Crc32Mpeg2

import rospy
from geometry_msgs.msg import Twist

ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
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
    rospy.init_node('serial_control', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber('/cmd_vel', Twist, callback)

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

    ser.reset_output_buffer()
    ser.reset_input_buffer()
    
    rate = rospy.Rate(10)
    rate.sleep()
    
    global linear_vel
    global angular_vel

    linear_vel = 0.0
    angular_vel = 0.0

    crc_tx = 0
    dim_rx = struct.calcsize('=HHLLL')

    while (not rospy.is_shutdown()):

        ## Transmit
        # convert to bytes and send
        vel_msg = struct.pack('<ff', linear_vel, angular_vel)
        ser.write(vel_msg)
        # calculate crc on byte buffer
        crc_tx = Crc32Mpeg2.calc(vel_msg)
        # comvert crc to bytes and send
        crc_tx_pack = struct.pack('<L', crc_tx)
        ser.write(crc_tx_pack)
        
        rospy.logdebug(vel_msg)

        ## Receive
        # read from serial
        buffer_rx = ser.read(12)
        crc_rx = ser.read(4)
        status_msg = struct.unpack('=HHLL', buffer_rx)
        crc_rx = struct.unpack('=L', crc_rx)[0]
        # calculate crc and check if ok
        crc_rx_calc = Crc32Mpeg2.calc(buffer_rx)
        if (crc_rx_calc != crc_rx):
            rospy.logerr("Error on rx (incorrect CRC)")
        
        rospy.logdebug(status_msg)
        rate.sleep()

def reset_st():
    ser.dtr = 1
    time.sleep(1)
    ser.dtr = 0

rospy.on_shutdown(reset_st)

if __name__ == '__main__':
    otto_serial_driver()
