#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013
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
roslib.load_manifest('cutter_localization')
import rospy
import tf
import math
import random
from sensor_msgs.msg import LaserScan

class LidarMakeSnow:
    def __init__(self):
        # Start the node
        rospy.init_node('lidar_make_snow')
        topic_in = rospy.get_param('~topic_in','/scan')
        topic_out = rospy.get_param('~topic_out','/base_scan')
        self.lambdashort = rospy.get_param('~lambda_short',0.03)
        
        rospy.Subscriber(topic_in, LaserScan, self.LidarCallback)
        self.pub = rospy.Publisher(topic_out, LaserScan)
        
        rospy.spin()
        
    
    def LidarCallback(self, msg):
        print "Lidar Received"
        newranges = []
        for dist in msg.ranges:
            d = -math.log(random.random()/self.lambdashort)/self.lambdashort
            if d > 0 and d < dist:
                dist = d
            newranges.append(dist)
        msg.ranges = newranges
        self.pub.publish(msg);

if __name__ == "__main__":
    try:
        LidarMakeSnow()
    except rospy.ROSInterruptException: pass

