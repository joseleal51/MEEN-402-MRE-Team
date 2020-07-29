# Multi-Agent Path Planning
### Texas A&M Mechanical Engineering - Senior Design

## Licensing Information: READ LICENSE

to be updated...

## Project Source Location
https://github.com/joseleal51/MEEN-402-MRE-Team

## Author & Contributor List

|Author|Contact|
|------|-------|
|Michiel Ashley III|miashley150@tamu dot edu|
|Manjeel Regmi|(-)|
|Jose Leal|(-)|
|Mohamad (Mo) Khachfe|mohamadkhachfe@gmail dot edu|
|Ahmad Tashkandi|(-)|
|Rawad Minkara|(-)|
|Henry Campbell|(-)|

## Project Description
As part of a senior design capstone project, the design team had to create a Multi-Robot Envirionment for Testing Multi-Agent Path Planning algorithms. The project was a two semester endevor, however the COVID-19 pandemic caused the project to adapt to simulation mid-way through the first semester. Before the pandemic, the design team worked with physical robots and a motion tracking system in the Advanced Robotics and Manufactuing Lab. See the ![Mechanical Engineering Research Labs](https://engineering.tamu.edu/mechanical/research/laboratories-and-groups.html). After the pandemic, the design team created python and Gazebo simulaitons.

### Turtlebot 2 
When working in the lab, ROS was used to control the robots. Each robot had an on-board computer and power-supply which communicated via WiFi to a central computer. Additionally, the design team began to set up a motion tracking system by Motive, called Opti-Track.  
Videos and pictures to be added...
 
### Python Simulation
The first part of the project is a python simulaiton that creats a grid world with robot start-points, end-points and obstacle points. Any world size, number of robots or obstacles can be defind. The simulation will plan each robot's path with the A* search algorithm only accounting for the world size and obstacle locations. Then collisions are detected by checking if two paths want to occupy the same point at the same time. Currently, the ntersection huristic is to make the robot(s) that has the lower priority in the projected collision to wait at its start point until the collison will not happen.

![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/show_collision_FINAL_NO.gif?raw=true)

![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/simulation_outputs/show_collision_FINAL_YES.gif?raw=true)

### Gazebo Simulation
The second part of the project is to demonstrate the environment and robot set-up in Gazebo to show how the robots move from one configuration to the final configuration.  
More info to be added...
