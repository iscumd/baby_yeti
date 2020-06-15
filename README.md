# baby_yeti
senior design project for ISC learning test bench

Contains:

Submodule for isc_joy
  -need to edit default topic names

PI controller based on ROS PI node

Sabertooth_2x12_motorcontroller:
  -supports different communication protocols,
  -python,
  -analog version: not working,
  -serial version: new version, needs to be tested with motorcontroller and motors

TODO:

Verify motorcontroller:
  -proper values written to pin 8 (Tx)?,
  -need NVIDIA_gpio libraries?
  
Add support for Intel RealSense T265,
Add support for Hokuyo LIDAR ust-10lx

Add dependencies to the CMakeLists.txt

Create folders for the LiDAR and RealSense
