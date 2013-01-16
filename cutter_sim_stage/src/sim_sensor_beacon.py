#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, EJ Kreinar
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('cutter_sim_stage')
import rospy
from numpy import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from cutter_msgs.msg import Beacon
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import rotation_matrix

import math, numpy, random
    
class SimBeacon():    
    def __init__(self):
        rospy.init_node('sensor_sim_beacon')
        
        # Parameters for beacon and receiver location
        # TODO: get the parameters from the beacon transforms!
        self.beacon_x = 0.0 # beacon location
        self.beacon_y = 5.0
        self.xoff = 0.0    # receiver offset from base_link
        self.yoff = 0.15
        
        # Sensor parameters
        self.std = 0.0
        
        # Set up Publishers and Subscribers
        self.pub = rospy.Publisher("/cwru/beacon",Beacon)
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.beaconCallback)
        
        # Loop ROS
        rospy.spin()

    def beaconCallback(self,data):
        # rospy.loginfo('callback called')
        beacon_msg = Beacon();
        
        # Rotation value
        rot = math.pi/2

        # Rotate the base pose
        origin, zaxis = (0, 0, 0), (0, 0, 1) 
        Rz = rotation_matrix(rot, zaxis, origin)
        pt = numpy.matrix((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 0),dtype=numpy.float64).T
        xy = Rz*pt

        # Add the angle rotation
        oldAng = data.pose.pose.orientation
        ang = euler_from_quaternion([ oldAng.x, oldAng.y, oldAng.z, oldAng.w ])
        ang = ang[0], ang[1], ang[2] + rot

        # Offset the base_pose_ground_truth by the specified xoff, yoff
        sensor_x = xy[0] + self.xoff*math.cos(ang[2]) - self.yoff*math.sin(ang[2])
        sensor_y = xy[1] + self.xoff*math.sin(ang[2]) + self.yoff*math.cos(ang[2])
        dist_x = sensor_x - self.beacon_x;
        dist_y = sensor_y - self.beacon_y;
        dist_true = math.sqrt(dist_x**2 + dist_y**2)
        dist = dist_true + random.gauss(0,self.std)
        print "testing"
        print xy
        print sensor_x, sensor_y
        print self.beacon_x, self.beacon_y
        print dist_x, dist_y
        print dist_true
        print dist

        # Populate the message
        beacon_msg.header.stamp = data.header.stamp
        beacon_msg.header.frame_id = "base_ranger_1"
        beacon_msg.beacon_frame = "beacon_1"
        beacon_msg.range = dist

        # Publish
        self.pub.publish(beacon_msg)


if __name__ == "__main__":
    try:
        SimBeacon()
    except rospy.ROSInterruptException: pass

