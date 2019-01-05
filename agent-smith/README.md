## Install dependencies

```bash
sudo apt-get install ros-kinetic-hector-slam ros-kinetic-map-server ros-kinetic-socketcan-bridge ros-kinetic-smach ros-kinetic-opencv3 libeigen3-dev ros-kinetic-navigation ros-kinetic-robot-localization ros-kinetic-teb-local-planner ros-kinetic-gmapping libpcl-conversions-dev ros-kinetic-pointcloud-to-laserscan ros-kinetic-rplidar-ros
```

In the upper level directory that stores your `catkin_ws` directory:
```bash
git clone https://github.com/FRC900/navXTimeSync.git
cd navXTimeSync
git checkout 3a935ceb
```

In the terminal, from the agent-smith directory: 

```bash 
git submodule init 
git submodule update
```

If you have an NVidia Graphics Card:
- Install the CUDA 9.1 package from the NVidia website
- Install the ZED Camera SDK
- Add the following to your bash.rc file:

```bash
# CUDA
export PATH=/usr/local/cuda-9.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-9.1/lib64:$LD_LIBRARY_PATH
```

In order to compile on a machine that doesn't have an NVidia Graphics Card:
- Install the CUDA 9.1 package from the NVidia website (do NOT accept the drivers, only the development package)
- Install the ZED Camera SDK
- Add the following to your bash.rc file:

```bash
# CUDA FAKE OUT
export PATH=/usr/local/cuda-9.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-9.1/lib64:$LD_LIBRARY_PATH
export CUDA_LIB_PATH=/usr/local/cuda/lib64/stubs/
```


## Build

From `catkin_ws` directory:
```bash
catkin_make
```

## Launching things

Use `roslaunch --screen <package_name> <launchfile_name>`.

## Run map generator

With lidar and odometry data: 
```bash
roslaunch xbot_nav xbot_nav_no_map.launch
```

With lidar data only: 
```bash
roslaunch xbot_map_gen hector_map_with_lidar.launch 
```

## Save maps
```bash
rosrun map_server map_saver -f /tmp/my_map
```
