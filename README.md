# Otto serial driver

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

## How to 

Use ```rosdep``` to install the dependencies needed:
```bash
rosdep install otto_serial_driver
```

In the ```launch``` folder you can find the launch files, launch ```otto_serial_driver.launch``` to start the communication with Otto, using the parameters specified in ```config/firware_parameters.yaml```.

**Important**: to be able to open the serial port, your user should be added to the `dialout` group:

```bash
sudo useradd -aG dialout $USER
```

Then logout, and log back in.
