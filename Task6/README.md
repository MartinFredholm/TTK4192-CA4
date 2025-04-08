# How to do task 6

### 1. Make world map
The map has been made by Øystein, and only needs to be placed in the correct folder. 

Comands to run to add this to the correct folder:
```
cp -r ttk4192_world ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
cp turtlebot3_ttk4192.world ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
```

Or manually:
1. Add ttk4192_world folder into the folder ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models folder.
2. Add turtlebot3_ttk4192.world into the folder ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds folder.

### 2. Create launch file
The launch file has been given, and must be placed in the correct file. 

Command to run
```
cp turtlebot3_ttk4192.launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
```

### 3. Run gazebo to see if world has been made

```
cd catkin_ws && catkin_make
export TURTLEBOT3_MODEL=waffle_pi
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch
```
