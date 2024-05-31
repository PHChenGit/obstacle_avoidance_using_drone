## Step 1 Install ROS noetic, PX4, mavros, gazebo 11


## Step 2 Install dependecy packages
### Step 2-1 Install rtabmap

rtabmap github: https://github.com/introlab/rtabmap_ros

```
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

then go to your .bashrc (or .zshrc)
Add the following line to bottom
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu
```
### Step 2-1 Install realsense depth camera SDK

realsense github: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

## Step 3 Launch gazebo with iris and depth camera, also launch rviz

1.  open a terminal, then
```
roscore
```

2. open another terminal
```
roslaunch demo.launch
```

3. (optinal) open QGroundControl

## Step 4. Launch rtabmap
```
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/iris/camera/depth/image_raw \
    rgb_topic:=/iris/camera/rgb/image_raw \
    camera_info_topic:=/iris/camera/rgb/camera_info \
    approx_sync:=false \
    rtabmapviz:=true
```