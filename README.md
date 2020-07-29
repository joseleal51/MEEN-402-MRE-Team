# Multi-Agent Path Planning
### Texas A&M Mechanical Engineering - Senior Design

## Licensing Information: READ LICENSE

to be updated...

## Project Source Location
https://github.com/joseleal51/MEEN-402-MRE-Team

## Author & Contributor List

|Author|Contact|
|------|-------|
|Michiel Ashley III|(miashley150@tamu dot edu)
|Manjeel Regmi|(-)|
|Jose Leal|(-)|
|Mohamad (Mo) Khachfe|(mohamadkhachfe@gmail dot edu)|
|Ahmad Tashkandi|(-)|
|Rawad Minkara|(-)|
|Henry Campbell|(-)|

## Project Description

The first part of the project is a python simulaiton that creats a grid world with robot start-points, end-points and obstacle points. Any world size, number of robots or obstacles can be defind. The simulation will plan each robot's path with the A* search algorithm only accounting for the world size and obstacle locations. Then collisions are detected by checking if two paths want to occupy the same point at the same time. Currently, the ntersection huristic is to make the robot(s) that has the lower priority in the projected collision to wait at its start point until the collison will not happen.

![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/show_collision_FINAL_NO.gif?raw=true)

![alt text](https://github.com/joseleal51/MEEN-402-MRE-Team/blob/master/show_collision_FINAL_YES.gif?raw=true)

The second part of the project is to demonstrate the environment and robot set-up in Gazebo to show how the robots move from one configuration to the final configuration.  
More info to be added...
