# agent-smith-simulator
The 2018 Robot Simulation

Make certain you've already cloned the Gazebo2018 repo in the proper location (~/.gazebo/models)

Prereqs:
```bash
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

To run:
```bash
roslaunch agent-smith-simulator/agent-smith_gazebo/launch/agent-smith-sim.launch
roslaunch agent-smith-simulator/agent-smith_control/launch/agent-smith-control.launch
```

To control commanded velocities from the UI, start RQT, select Plugins -> Topics -> Message Publisher, and add and check the 6 /xbot/*\_velocity\_controller/command topics
```bash
rosrun rqt_gui rqt_gui
```
