# Testing workspace for visualizing Astrobee 

First create a ROS workspace to work from and clone the repo:
```
    mkdir -p ~/rviz_ws/src && cd ~/rviz_ws/src
    git clone https://github.com/acauligi/bee_viz.git
```



Next initialize your workspace
```
    cd ~/rviz_ws && catkin init
```

Copy over the *scp* directory from the [Astrobee](https://github.com/acauligi/astrobee/tree/traj_opt/mobility/planner_scp) repo and place it in the *rviz_ws* workspace. In *CMakeLists.txt*, set the NASA option to OFF.

Next build the project: 
```
    catkin_make && source devel/setup.bash
```

Finally, to run the project:
```
    source ~/rviz_ws/devel/setup.bash
    roslaunch bee_viz ff_visualizer.launch
```
