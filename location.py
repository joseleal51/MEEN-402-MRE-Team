# -*- coding: utf-8 -*-


"""
Created on Mon Jun 29 16:26:18 2020
@author: Manjeel RR
--> This code tells you the location of the closest obstruction to 
the Robot
"""
"""
In terminal - 
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_teleop keyboard_teleop.launch

chmod +x location.py --> makes python file executable
rosrun location_m location.py
"""
import rospy
from nav_msgs.msg import Odometry
import math
from location_m.msg import LandmarkDistance

def distance(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    return math.sqrt(xd^2 + yd^2)

class LandmarkMoniter(object):
    def __init__(self, pub, landmarks): #pub = publisher
        self._pub = pub
        self._landmarks = landmarks

    def callback(self, msg):
        x = msg.pose.pose.position.x # x position of robot
        y = msg.pose.pose.position.y
        closest_name = None
        closest_distance = None
        for l_name, l_x, l_y in self._landmarks: #comparision of robot's location and obstruction's 
            dist = distance(x, y, l_x, l_y)
            if closest_distance is None or dist < closest_distance:
                closest_name = l_name
                closest_distance = dist
        ld = LandmarkDistance()
        ld.name = closest_name
        ld.distance = closest_distance
        self._pub.publish(ld) # .publish() publishes message, see line 57

def main():
    rospy.init_node('location_m')
    landmarks = []                         #location of  each landmark
    landmarks.append(("Cube", 0.31, -0.99));
    landmarks.append(("Dumpster", 0.11, -2.42));
    landmarks.append(("Cylinder", -1.14, -2.88));
    landmarks.append(("Barrier", -2.59, -0.83));
    landmarks.append(("Bookshelf", -0.09, 0.53));
    
    pub = rospy.Publisher('closest_landmark', LandmarkDistance, queue_size = 10) #publishing to the topic /closest_landmark
                            #publisher topic, msg type, queue size         
    monitor = LandmarkMoniter(pub, landmarks)
    
    rospy.Subcriber("/odom", Odometry, monitor.callback) #subscribes to the topic that tells us where the robot is (/odom)
                    #name of topic, msg class, callback
    rospy.spin() #allows the program continuously run
    
    #in terminal: rostopic echo /closest_landmark ; the code is publishing there
 
if __name__ == '__main__':
    main()