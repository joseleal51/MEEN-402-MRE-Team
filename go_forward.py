#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# An example of TurtleBot 2 drawing a 0.4 meter square.
# Written for indigo

import rospy
from geometry_msgs.msg import Twist
from math import radians

class GoStraight():
    def __init__(self, distance):
        # initiliaze
        rospy.init_node('go_forward', anonymous=False)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Distance to travel
        self.distance = distance
     
	# 5 HZ
        r = rospy.Rate(5)

	# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
	# by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
        #turn_cmd = Twist()
        #turn_cmd.linear.x = 0
        #turn_cmd.angular.z = radians(45); #45 deg/s in radians/s

        #while not rospy.is_shutdown():
        # go forward 0.4 m (2 seconds * 0.2 m / seconds)
        rospy.loginfo("Going Straight "+str(self.distance)+" meter.")
        for x in range(0, int(round(self.distance*10/0.4))):
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Going Forward "+str(self.distance)+" meter.")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        distance = 1 # meter
        GoStraight(distance)
    except:
        rospy.loginfo("node terminated.")