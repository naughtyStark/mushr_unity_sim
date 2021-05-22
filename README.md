# mushr_unity_sim

Mushr_unity_sim provides provides a drop-in replacement for the default state simulator (racecar_state.py) used in mushr_sim. The unity backend allows the simulation of things such as wheel-slippage, body-roll, and so on. This is better suited for those that want to test their control algorithms after they have been tested in the default mushr simulation environment (but you could develop directly with this too).

Below are the install and run instructions, but the best way to get started is to checkout the this [tutorial](https://mushr.io/tutorials/mushr_unity_sim/). Detailed run instructions should be in a tutorial, this serves as a quick reference

Author:
[Sidharth Talia](https://www.sidharthtalia.com/)

#### User Prerequisites:
1) Familiarity with ROS, python.
2) Completed the MuSHR [quickstart tutorial](https://mushr.io/tutorials/quickstart/) up to the simulation part. (Please use the sandbox map)

### Installation:

#### Cloning the repository::
``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/naughtyStark/mushr_unity_sim.git
$ cd ~/catkin_ws
$ catkin_make
```
The last catkin_make command is necessary.

Python dependencies: 
External packages this project depends on: numpy 
``` bash
$ cd ~/catkin_ws/src/mushr_unity_sim
$ pip install -r requirements.txt
```

### Running the base example:
Enter the following commands in the terminal:
``` bash
$ roslaunch mushr_unity_sim unity_multi.launch
```

You should see 4 cars by default. The poses of these cars are set by the pose_init.py file. The car's initial pose is set the same way as done for the default mushr_sim; by publishing a message on the /car_name/initialpose topic. 

Since it has the same interface as the default mushr_sim multi_teleop.launch, you should be able to drive the cars with the WASD keys.

Note that collisions between the cars will also be simulated.

Launching the simulator seperately:
``` bash
$ ./catkin_ws/src/mushr_unity_sim/unity/mushr_unity_sim.x86_64 -batchmode
```
The tag `batchmode` is to make sure the simulator runs in headless mode. If you were able to install nvidia graphics drivers on your linux without tanking the display stuff (god bless) you can run it in normal mode. It's not much to look at.

Launching rviz separately:
``` bash
$ rviz -d ~/catkin_ws/src/mushr_unity_sim/rviz/mushr_unity_sim.rviz
```


### API
When using namespace in the launch file (as done in unity_multi.launch) the topics will have a prefix attached to them corresponding to the name given to the car. We're calling that name "car_name" for the sake of explanation.

### Publishers
Topic | Type | Description
------|------|------------
`/car_name/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of the car
`/car_name/car_odom` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)| odometry of the car. Velocity is in body frame!
`/car_name/joint_states` | [sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)| wheel positions
`/car_name/car_imu` | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)| Imu data (body frame)

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



