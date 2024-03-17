# NUSLAM
Author: Shail Dalal

## Quickstart
1. Use `ros2 launch nuslam slam.launch.xml robot:=nusim cmd_src:=teleop` to start the simulation in RVIZ with a teleop twist keyboard
2. Use `ros2 launch nuslam slam.launch.xml robot:=nusim cmd_src:=circle` to start the simulation in RVIZ with the circle node to make the robot move in a circle
    - Use `ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.1, radius: 0.1}` to start the circular motions of the robot in rviz
3. Here is a picture of the turtlebot in the arena
![Screenshot from 2024-03-01 00-37-51](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/ec7e709b-89f2-41ef-a939-5c6b4de1b59a)

### Launch slam with known data association:
`ros2 launch nuslam slam.launch.xml`

This launches the Slam in simulation with known data. The red robot is ground truth, the blue is odometry estimation and green is Slam estimation. Here the location and ids of the markers are known. The yellow blinking markers are noisy marker positons. 

### Launch SLAM with unknown data association (using lidar data):
`ros2 launch nuslam unknown_data_assoc.launch.xml`

This launches the Slam in simulation with unknown data association. Here the simulated lidar is used to get positions of the markers. The points are then read in as clusters and then fit to a circle. This launches the simualtion with a teleop twist keyboard which publishes cmd_vel

## Simulation Results

Below is the video of the slam running with unknown data association. It can be seen that after collision the error increases a lot.

[Screencast from 03-17-2024 12:43:25 PM.webm](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/f0322fe8-96bb-495b-9d64-e657f269c77d)


At the end of loop closure, the position of the green and blue wrt to red are as follows

| robot | x difference | y difference | 
| ------| ------------- | --------------- |
| red | 0.00024 m | -0.0006 m |
| green (SLAM) | 0.0076 m | 0.003 m |
| blue (odometry) | 0.330 m| -0.482 m |


## Real Turtlebot

Steps:
1. Connect to the Turtlebot
2. Run the launch file on the turtlebot using `ros2 launch nuslam  turtbot_bringup.launch.xml`
3. Run the launchfile on the laptop for visaulaization `ros2 launch nuslam pc_bringup.launch.xml`


Below is the video of the turtlebot running in real life

[!realRobot](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/8a926828-82f4-4df8-8d91-104561dd03d7)

Here is the screenshot of the rviz at the end of the run:

![Screenshot from 2024-03-17 13-13-15](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/901ae980-1e11-4986-b0da-291219d76bd5)


The real tutlebot had a lot of noise. This let to the 3 out of 4 obstacles being recognised at the end as the 4th jumped locations at the end


| robot | x coordinate | y coordinate | 
| ------| ------------- | --------------- |
| green (SLAM) | -0.015 m | -0.084 m |
| blue (odometry) | 0.05 m | -0.13 m |