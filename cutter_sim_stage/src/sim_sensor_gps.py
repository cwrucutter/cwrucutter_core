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
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from gps_common.msg import GPSStatus
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import rotation_matrix

import math, numpy, random

class SimSensorGPS:
    def __init__(self):
        rospy.init_node('sensor_sim_gps')
        self.pub = rospy.Publisher("/gps_pose",PoseStamped)
        self.pubStat = rospy.Publisher("/gps_status",GPSStatus)
        
        # declare messages
        self.gps_msg = PoseStamped()
        self.gps_stat = GPSStatus()
        # Mimic GPS status message
        self.gps_stat.satellites_used = 7
        self.gps_stat.satellites_visible = 9
        self.gps_stat.position_source = 50
        
        #setup subscriber
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.callback)
        
        #spin
        rospy.spin()
        
        
    def callback(self, data):
        # rospy.loginfo('callback called')
        
        # Set parameters
        # TODO: get the parameters from the gps transform!!!
        xoff = -0.45
        yoff = 0.0
        std = .03
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
        
        # Offset the GPS reading by the lever arm offset
        x = xy[0] + xoff*math.cos(ang[2]) - yoff*math.sin(ang[2])
        y = xy[1] + xoff*math.sin(ang[2]) + yoff*math.cos(ang[2])
        #x = xy[0] + xoff*math.cos(ang[2])
        #y = xy[1] + yoff*math.sin(ang[2])
        gps_x = x + random.gauss(0,std)
        gps_y = y + random.gauss(0,std)
        print "testing"
        print xy
        print x,y
        print gps_x, gps_y    

        # Populate the angle
        # Note... GPS doesnt really return a heading!! So we cant use to localize... rememmmmberrrrr
        # gps_msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(*ang))
        self.gps_msg.pose.orientation = Quaternion(*quaternion_from_euler(*ang))
        
        # Populate x,y
        self.gps_msg.header.stamp = data.header.stamp
        self.gps_msg.header.frame_id = "map_gps"
        #self.gps_msg.pose.pose.position.x = gps_x
        #self.gps_msg.pose.pose.position.y = gps_y
        self.gps_msg.pose.position.x = gps_x
        self.gps_msg.pose.position.y = gps_y

        # Publish
        self.pub.publish(self.gps_msg)
        self.pubStat.publish(self.gps_stat)


if __name__ == "__main__":
    try:
        SimSensorGPS()
    except rospy.ROSInterruptException: pass

