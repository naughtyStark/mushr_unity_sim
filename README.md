# unity_backend_mushr

This repository provides a physics engine for use with the MuSHR(link) sim environment. While the default mushr_sim simulation works well for debugging basic operations, a better physics back-end may be used before stepping into the real world (or the entire development may be done directly with this physics backend if you want). The physics backend here leverages the unity game engine to simulate the motion of the MuSHR car. This results in a simulation that considers wheel slippage, body roll, the inertia of the speed control system, and so on. 

Thus, it provides a drop-in replacement for the default state simulator (racecar_state.launch). The simulator itself is derived from the donkey-simulator made by Tawn Kramer(check name) and is used by the DIYRobocars community. Note that this backend does not provide camera images and only acts as a physics backend. 

#### User Prerequisites:
1) Familiarity with ROS, python.
2) Completed the MuSHR [quickstart tutorial](https://mushr.io/tutorials/quickstart/) up to the simulation part.

### Installation:

python dependencies: 
1) numpy
```
pip install numpy
```
#### Cloning:
```
cd ~/catkin_ws/src
git clone https://github.com/naughtyStark/unity_backend_mushr.git
cd ~/catkin_ws
catkin_make
```
The last catkin_make command is necessary to make the system recognize unity_backend as a valid ros package.

### Running the base example:
terminal command:
```
cd ~/catkin_ws/src/unity_backend_mushr/unity
./donkey_sim.x86_64 -batchmode
```
If you have nvidia driver support on your linux machine (god bless), you can run it without the "-batchmode" tag. The tag makes the sim run in the headless mode which allows for higher fps if you don't have said driver support.

In a new tab:
```
roslaunch unity_backend unity_multi.launch
```
In another new tab:
```
rosrun rviz rviz
```
To visualize the simulation in rviz, use the rviz config in 
```
~/unity_backend/rviz/unity_backend.rviz
```
You should see 4 cars by default. The poses of these cars are set by the pose_init.py file. The car's initial pose is set the same way as done for the default mushr_sim; by publishing a message on the /car_name/initialpose topic. 

As it has the same interface as the default mushr_sim multi_teleop.launch, you should be able to drive the cars with the WASD keys.

Note that collisions between the cars will also be simulated (try not to collide though. The purpose was simply to make the simulation somewhat interesting). 

### API
When using namespace in the launch file (as done in unity_multi.launch) the topics will have a prefix attached to them corresponding to the name given to the car. We're calling that name "car_name" for the sake of explanation.

### Publishers
Topic | Type | Description
------|------|------------
`/car_name/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of the car
`/car_name/car_odom` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)| odometry of the car
`/car_name/joint_states` | [sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)| wheel positions
`/car_name/car_imu` | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)| Imu data

### Subscribers
Topic | Type | Description
------|------|------------
`/car_name/mux/ackermann_cmd_mux/output` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)| steering and speed control commands to be sent to the car
`/car_name/initialpose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| initial position of the car


## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.

The ROS-wrapper around the donkey-sim for integration into the MuSHR project was created by [Sidharth Talia](https://www.sidharthtalia.com/)



