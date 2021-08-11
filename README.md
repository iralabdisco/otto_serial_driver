# otto_serial_driver

ROS driver for Otto.


## Description

The node ``` otto_serial_driver.py ``` is used to communicate with Otto.
It performs the following tasks:

- Open the serial port
- Reset the MCU
- Transmit the startup configuration
- Listen for messages on cmd_vel, and transmit the last cmd_vel received every 100ms to the MCU
- Receives data from Otto, publish it on ```otto_ticks``` topic
- Publishes odometry and tf transform

The node ``` otto_pid_tuning.py ``` is used to tune PID parametes.
It subscribes to ```cmd_vel``` and ```otto_ticks``` and publishes a ```otto_pid_tuning_val``` message.
You can use ```rqt_plot``` to plot the values and tune your PID.


## How to 

Use ```rosdep``` to install the dependencies needed.

In the ```launch``` folder you can find the launch files, launch ```otto_serial_driver.launch``` to start the communication with Otto, using the parameters specified in ```config/firware_parameters.yaml```.
