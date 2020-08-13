# Multi-Agent Path Planning
### Texas A&M Mechanical Engineering - Senior Design

## Licensing Information:

Anyone is free to copy, modify and sell the source code in this repository.

## Project Source Location
https://github.com/joseleal51/MEEN-402-MRE-Team

## Author & Contributor List

|Author|Contact|
|------|-------|
|Michiel Ashley III|miashley150@tamu dot edu|
|Manjeel Regmi|(manjeelregmi@tamu dot edu)|
|Jose Leal|(-)|
|Mohamad (Mo) Khachfe|mohamadkhachfe@gmail dot edu|
|Ahmad Tashkandi| ahmad.tashkandi.1@kaust dot edu dot sa|
|Rawad Minkara|(-)|
|Henry Campbell|(-)|

## Project Description
As part of a senior design capstone project, the design team had to create a Multi-Robot Envirionment for Testing Multi-Agent Path Planning algorithms. The project was a two semester endevor, however the COVID-19 pandemic caused the project to adapt to simulation mid-way through the first semester. Before the pandemic, the design team worked with physical robots and a motion tracking system in the Advanced Robotics and Manufactuing Lab. See the ![Mechanical Engineering Research Labs](https://engineering.tamu.edu/mechanical/research/laboratories-and-groups.html). After the pandemic, the design team created python and Gazebo simulaitons.

### Turtlebot 2 
When working in the lab, ROS was used to control the robots. Each robot had an on-board computer and power-supply which communicated via WiFi to a central computer. Additionally, the design team began to set up a motion tracking system by Motive, called Opti-Track.  
Videos and pictures to be added...
 
### Python Simulation
The first part of the project is a python simulaiton that creats a grid world with robot start-points, end-points and obstacle points. Any world size, number of robots or obstacles can be defind. The simulation will plan each robot's path with the A* search algorithm only accounting for the world size and obstacle locations. Then collisions are detected by checking if two paths want to occupy the same point at the same time. Currently, the ntersection huristic is to make the robot(s) that has the lower priority in the projected collision to wait at its start point until the collison will not happen.

Before the collision avoidance algorithm is implemented:
![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/show_collision_FINAL_NO.gif?raw=true)


After the collision avoidance algorithm is implemented:
![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/show_collision_FINAL_YES.gif?raw=true)


## Simulation Flow Chart
![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/simulation_flowchart-1-1.jpg?raw=true)

## Collision Avoidance Flow Chart
![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/collision_avoid_algo.png?raw=true)



### Gazebo Simulation
The second part of the project is to demonstrate the environment and robot set-up in Gazebo to show how the robots move from one configuration to the final configuration.

# The Environment 

The environment, shown in the image below, was designed on the basis of allowing for different path planning algorithms to take place. 

 ![Final_Gazebo_Environment](https://user-images.githubusercontent.com/66921594/90177151-bd952d00-dd6f-11ea-9b00-482aa14b3d31.jpg)

There is an obstacle with a cavity in it that allows one or two robots to fit in it. This allows for robots to go inside the cavity to await instruction or for another robot to pass, then follow. The environment’s design on gazebo was trivial to the team as there was very little knowledge and experience with Gazebo. Creating an environment itself was a problem as the team working on Gazebo believed that a package with sources and dependencies must have been made to launch a world. This was later proved to be unnecessary but still critical information for when one wishes to start a whole new environment. It is true that you need to create a package and list the proper dependencies: these are packages that allow for nodes to be created and wrappers for different coding languages i.e. rospy. Launching a world in Gazebo requires a launch file within a certain package to be specified. The reason it was not so necessary for this specific case is because a launch file is already provided where a Kobuki TurtleBot is already provided, all that is needed is a saved world file to be specified with the launch file so that the saved environment can be launched and testing with the TurtleBot can begin. The line of code is shown below: 

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/path to world file/ 

This input would load the saved environment after installing the turtlebot_gazebo packages necessary for your ROS distribution – the team used ROS Kinetic.  It is recommended to make your own package to be able to easily share your world and launch file at your own settings, however it’s not entirely necessary. 

# The Path Planning 

ROS has built in path planning. This path planning is responsible for autonomous driving. The path planning is housed in a ROS program called RVIZ. In order to access RVIZ, an environment was needed. There was a laser attached to the robot. RVIZ allowed the TurtleBot to utilize its laser to create a 2D representation of the environment that the TurtleBot was in. The robot could only move in the x-y plane, therefore, the map served as the facilitator in autonomous driving. The following figure is a map of the environment for this project. 

![MAP](https://user-images.githubusercontent.com/66921594/90177627-6fccf480-dd70-11ea-972c-78deef39b536.png)

In the above figure, the TurtleBot is the top-right corner. The rainbow square around the robot is an illustration of the range of the laser. The robot must be driven with continuous keyboard input with the “teleop” function, so that the laser mapped out the environment.   

After the map is made, three terminal commands were necessary to be issued. One command was necessary to launch the environment; that was done using the “roslaunch” function. Another command was necessary to set up the communication between the environment and RVIZ. When this terminal was executed, it would say something along the lines of “odom is successfully connected”. Odom is the topic that tells the user where the robot is in the environment. This in conjunction with the map which tells the user where the barriers are in the environment provided the user with all the information necessary to use the autonomous driving within RVIZ. A third and final command launched RVIZ. The following figure displays a video of ROS’s autonomous driving. 

  ![QR_Code](https://user-images.githubusercontent.com/66921594/90177783-a9056480-dd70-11ea-9c7f-ead8539a0b67.jpg)

This video demonstrates the hesitation of the robot when it tried to determine the path it needed to take in order to reach the assigned goal. It took the robot one minute and fourteen seconds of real time to reach a destination that was very close to it; the robot would constantly get stuck and the command to tell the robot to go to the assigned goal had to be reissued. This demonstrates the need for a better path planning algorithm, and that’s where custom path planning developed by the Python team comes into play. 

