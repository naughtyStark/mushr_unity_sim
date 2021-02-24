# ros_donkey_sim

A ROS-port of the famous [donkey-sim](https://github.com/tawnkramer/sdsandbox) used by the DIY-robocar community. This version is actually a ROS-port of the donkey sim modified for the MuSHR project at The University of Washington.

### Installation:
```
pip install gym-donkeycar
cd ~/catkin_ws/src
git clone https://github.com/naughtyStark/ros_donkey_sim.git
cd ~/catkin_ws
catkin_make
```

### Running:
terminal command:
```
cd ~/catkin_ws/src/ros_donkey_sim/unity
./donkey_sim.x86_64 -batchmode
```
If you have nvidia driver support on your linux machine (god bless), you can probably run it without the "-batchmode" tag. The tag makes the sim run in the headless mode
which allows for higher fps if you don't have said driver support.

In a new tab:
```
roscore
```
In another new tab:
```
rosrun ros_donkey_sim run_sim_ros.py
```
Right now this will set the car in an open area (no obstacles) and provide the car's state on the following topics:
```
/car1/car_odom
/car1/car_imu
```
Note that the linear velocity in the odom messages is in the global reference frame and not the local reference frame (I may change this later if needed).
The node subscribes to the AckermannDriveStamped type message: 
```
/car1/mux/ackermann_cmd_mux/input/navigation
```


## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.




