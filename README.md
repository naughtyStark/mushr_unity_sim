# ros_donkey_sim

A ROS-port of the famous [donkey-sim](https://github.com/tawnkramer/sdsandbox) used by the DIY-robocar community. This version is actually a ROS-port of the donkey sim modified for the MuSHR project at The University of Washington. The purpose of this sim is to act as a physics engine. In contrast to other physics engines such as MuJoCo, this one is supposed to be free and open source (no licensing yaaaay!). The source code for the simulator will be shared soon as well. 

This physics engine bridges (or is supposed to bridge) the gap between the mushr-sim and the real world by providing more realistic physics simulations. At the moment only single agent simulation has been tested, however, multi-agent support is in the works. 

### Installation:
Installing the donkeycar specific gym backend
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
roslaunch unity_backend unity_backend.launch
```
Right now this will set the car in an open area (no obstacles) and provide the car's state on the following topics:
```
/car1/car_odom
/car1/car_pose
/car1/joint_states
/car1/car_imu
```
To visualize the simulation in rviz, use the rviz config in 
```
~/unity_backend/rviz/unity_backend.rviz
```
Note that the linear velocity in the odom messages is in the local reference frame/body-frame.
The node subscribes to the AckermannDriveStamped type message: 
```
/car1/mux/ackermann_cmd_mux/input/navigation
```
You can test it by using rostopic pub -r 10 (topic name and so on... ). Note that the car will stop if it doesn't receive a message for more than 1 second.


## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.




