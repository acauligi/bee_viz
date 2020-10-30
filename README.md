# Testing workspace for visualizing Astrobee 

First create a ROS workspace to work from and clone the repo:
    mkdir -p ~/rviz_ws/src && cd ~/rviz_ws/src
    git clone https://github.com/acauligi/bee_viz.git

Next initialize your workspace
    cd ~/rviz_ws && catkin init
    catkin_make && source devel/setup.bash
    roscore

In a separate terminal run
    source ~/rviz_ws/devel/setup.bash
    roslaunch bee_viz ff_visualizer.launch

In a separate terminal run
    source ~/rviz_ws/devel/setup.bash
    rosrun bee_viz render
