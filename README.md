# unity_backend_mushr

This repository provides a physics engine for use with the MuSHR(link) sim environment. While the default mushr_sim simulation works well for debugging basic operations, a better physics back-end may be used before stepping into the real world. The physics backend here leverages the unity game engine to simulate the motion of the MuSHR car. This results in a simulation that considers wheel slippage, body roll, the inertia of the speed control system, and so on. 

The unity backend provides a drop-in replacement for the default state simulator (racecar_state.launch). The simulator itself is derived from the donkey-simulator made by Tawn Kramer(check name) and is used by the DIYRobocars community. Unlike the original donkey-simulator, this simulator does not provide camera images. 


### Installation:

Assuming you already have the rospy and mushr_sim installed (if not, what are you doing here? Go and install that first (link) )
you'll also need to have:


Installing the donkeycar specific gym backend. We don't actually use the gym environment, we actually just need some helper code. The installation may be modified in the future to make this a self-contained project without any overhanging dependencies.
```
pip install gym-donkeycar
cd ~/catkin_ws/src
git clone https://github.com/naughtyStark/unity_backend_mushr.git
cd ~/catkin_ws
catkin_make
```

### Running:
terminal command:
```
cd ~/catkin_ws/src/unity_backend_mushr/unity
./donkey_sim.x86_64 -batchmode
```
If you have nvidia driver support on your linux machine (god bless), you can probably run it without the "-batchmode" tag. The tag makes the sim run in the headless mode which allows for higher fps if you don't have said driver support.

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
Note that the linear velocity in the odom messages is in the local reference frame/body-frame.

As it has the same interface as the default mushr_sim multi_teleop.launch, you should be able to drive the cars with the WASD keys.


## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.




