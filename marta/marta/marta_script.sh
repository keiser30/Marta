#!/bin/bash
# This script allow the initialization of all launch.
# Necessary fot the operation of marta.

clear

echo "The script starts now."

echo "HI, $USER!"
echo 
cd ..
cd ..
source ./devel/setup.bash
echo
echo "We will give permission to dynamixel and sensors"
echo 
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
sudo chmod 777 /dev/video0
echo

echo "We will activate the dynamixel and sensors"
echo 
MONITOR="bash -c \"source ./devel/setup.bash; roslaunch marta marta_monitor.launch; exec bash\""
CAMERA="bash -c \"source ./devel/setup.bash; roslaunch marta marta_camera.launch; exec bash\""
IMU="bash -c \"source ./devel/setup.bash; roslaunch marta marta_imu.launch; exec bash\""
gnome-terminal -e "$MONITOR"
sleep 2
gnome-terminal -e "$CAMERA"
sleep 2
gnome-terminal -e "$IMU"
sleep 15
echo


echo "We will activate the project"
echo 
rosrun marta marta_software
echo

