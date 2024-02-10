# NUTurtle Control

Author: Shail Dalal <br>
The package consists of 3 nodes  that control the movement and actions of a turtle robot. 

## Nodes

The nodes are as follows:
1.  `turtle_control_node`: This node is responsible for controlling the movement of the turtle robot. It uses the `turtlelib` kinematics library. This node can control both the simulation and real turtlebot3.
2. `odometry`:  This node is responsible for calculating the position of the turtle using the `diff_drive` class of the `turtlelib` library. It uses the `joint_states` and `wheel_cmd` to calculate the foward kinematics
3. `circle`: This node publishes a `cmd_vel` which will be responsible for the movement of the robot. it provides a `control` service which can be called as `ros2 service call /control nuturtle_control/srv/Control "{velocity: velocity, radius: radius}"` which sets the linear and angular velocity. It also provides `reverse` service as `ros2 service call /reverse std_srvs/srv/Empty` which makes the robot move in reverse and also a `stop` service as `ros2 service call /stop std_srvs/srv/Empty` which stops the robot movement.


## Launch File

The package has a launchfile which can be used to start simulation as well work with the real robot. <br>
It can be used as `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=X robot:=Y use_rviz:=Z` <br>

The `cmd_src` options are:
- `circle` - This starts the `circle` node which commands the  turtle bot to move in circles.
- `keyboard` - This starts `teleop_twist_keyboard` node to control the robot
- `none` - This will launch no nodes to control the robot

The `robot` options are :
- `nusim` - Launches the Nusim node which makes the robot in simulator mode
- `localhost` - Used to launch on the real robot
- `none` - doesnt launch any extra nodes

The  `use_rviz` options are:
- `true` - Starts Rviz for visualization
- `false` - Doesnt start rviz


## Quickstart for simulation
1. Use `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=nusim use_rviz:=true` to start the simulation in RVIZ with circle mode
2. Use `ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.1, radius: 0.1}` to start the circular motions of the robot in rviz

## Quickstart for  real robot
1. ssh into the robot with a ROS_DOMAIN_ID set by `ssh -oSendEnv=ROS_DOMAIN_ID user@remote`
2. from the local laptop, rsync the files compiled using `aarch64` architecture supported by the raspberry pi using `rsync -av --delete aarch64_install/ user@host:/path/to/remote/install`. Source the setup.bash from this install folder.
3. Use `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=localhost use_rviz:=false` on the turtlebot (remote).
**(Note: if the domain id is set right, you should see the nodes running from the host machine)
4. If you want to visualize on the host, you ca run `ros2 launch cmd_src:=none robot:=none use_rviz=true` to launch the rviz with blue robot.
5. From the host, run `ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.1, radius: 0.5}` and watch the robot create circles on the ground. 

The video below shows the movent of robot in a circle, using the `control`, `reverse` and `stop` sevices. To make the robot reach the exact start posiotn `teleop_twist_keyboard` is used:

[!CIRCLE](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/478255eb-6318-4356-bef3-168067404fe9)

### Final Robot location wrt Odom
The picture uploaded below show the location of links in RVIZ after completing couple of circle  movements. The psoition shows the slipping and error in the robot odometry.
![Screenshot from 2024-02-08 23-02-55](https://github.com/ME495-Navigation/slam-project-sdalal1/assets/80363654/cdbd73af-4c87-4386-8006-755c0855e82e)

The picture shows that displacement of base_footprint w.r.t odom in `x` and `y` is `(0.24,0.093)` which leads to the total error in odometry to be `~0.2579 m`