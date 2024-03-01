# NuSLAM

Author: Shail Dalal <br>
The node is used to simulate SLAM in RVIZ

## Quickstart
1. Use `ros2 launch nuslam slam.launch.xml robot:=nusim cmd_src:=teleop` to start the simulation in RVIZ with a teleop twist keyboard
2. Use `ros2 launch nuslam slam.launch.xml robot:=nusim cmd_src:=circle` to start the simulation in RVIZ with the circle node to make the robot move in a circle
    - Use `ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.1, radius: 0.1}` to start the circular motions of the robot in rviz
3. Here is a picture of the turtlebot in the arena
![Screenshot from 2024-03-01 00-37-51](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/ec7e709b-89f2-41ef-a939-5c6b4de1b59a)