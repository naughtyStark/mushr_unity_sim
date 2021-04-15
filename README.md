# unity_backend_mushr

This repository provides a physics engine for use with the MuSHR(link) sim environment. While the default mushr_sim simulation works well for debugging basic operations, a better physics back-end may be used before stepping into the real world (or the entire development may be done directly with this physics backend if you want). The physics backend here leverages the unity game engine to simulate the motion of the MuSHR car. This results in a simulation that considers wheel slippage, body roll, the inertia of the speed control system, and so on. 

Thus, it provides a drop-in replacement for the default state simulator (racecar_state.launch). The simulator itself is derived from the donkey-simulator made by Tawn Kramer(check name) and is used by the DIYRobocars community. Note that this backend does not provide camera images and only acts as a physics backend. 

### Installation:

#### Prerequisites:
1) ROS
2) mushr_sim (link to quickstart tutorial)

python dependencies: 
numpy
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

## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.




