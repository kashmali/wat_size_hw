# General
This is the [WatSize](https://watsize.github.io/) repo for the hardware, containing the electrical schematics, firmware, and relevant documentation

# Setup
To get your environment setup for the use of this repo follow these steps:

1. Configure OpenOCD
   +sudo apt-get install openocd
   $cd /usr/local/share
   $sudo ln -s /usr/share/openocd .
2. Get ST-Link
   +Follow instructions [here](https://github.com/texane/stlink)
3. get GCC
   +ensure that gdb-multiarch runs
   +ensure arm-none-eabi-gcc runs

# What does the HW do?
- Interfaces with LiDAR, motor controller, ethernet adapter, camera to provide pictures and point clouds of users
- Upon request from AWS server this device will take a picture, rotate a user at a constant RPM (PID controlled), and scan them with a 2D-lidar to form a point cloud and transmit all the data back to the AWS server

# Contact Info 
Aakash Mali
kashmali3313@gmail.com
