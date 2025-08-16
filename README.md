# Wall Follower (ROS Noetic)

This package implements a **Reactive Wall Following robot** in ROS Noetic.  
Features:
- Follows the right wall using LaserScan.
- Stops on detecting a colored marker using OpenCV.

## Build
```bash
cd ~/ros_projects_ws
catkin_make
source devel/setup.bash
