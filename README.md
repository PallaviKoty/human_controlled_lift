# human_controlled_lift

This is the ROS plugin for triggering the lift action based of the robot's position

Steps to use this plugin:

1. Clone common_msgs for melodic and compile it using the command
```
catkin_make --only-pkg-with-deps common_msgs
```
2. `source devel/setup.bash`
3. Compile the robot_position_plugin package using the command,
```
catkin_make --only-pkg-with-deps robot_position_plugin
```
4. `source devel/setup.bash`
5. Compile the elevator_plugin package using the command,
```
catkin_make --only-pkg-with-deps elevator_plugin
```
5. Launch the launch file using the command,
```
roslaunch elevator_plugin launch_elevator.launch
```
6. After Step 5, we can see gazebo is up with the lift model and a human
7. We can see the position of the model published on `/robot_pose`
8. To see the floor number, check on topic `/elevator`
9. Move the human within the recognizable range in front of the elevator to call the lift car to current floor
