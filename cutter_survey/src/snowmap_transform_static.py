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
roslib.load_manifest('cutter_survey')
import rospy
import tf
import math

# TODO: Improve the Surveying Procedure if we want!
#       1. Surveying can publish a message which grows 
#          each time a point is surveyed
#       2. Transformer node can be running before surveying, 
#          subscribe to that message, and then update the 
#          map -> snowmap transform every time a new point arrives
#       3. Subscribe to and Publish the gps_pose

class SnowmapTransformer:
    def __init__(self):
        # Start the node
        rospy.init_node('snowmap_tf_broadcaster')
        
        self.translate = (-3.0, -5.0, 0.0) 
        self.rotate = tf.transformations.quaternion_from_euler(0.0,0.0,-2.3 )
        
        rospy.loginfo('Transform successful! Publishing transform')
        # Initialize ROS transform broadcaster
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy. is_shutdown():
            br.sendTransform(self.translate,
                                 self.rotate,
                                 rospy.Time.now(),
                                 "snowmap",
                                 "map")
            rate.sleep()
        else:
            rospy.logerr('Transform was not successful. Leaving Node')


if __name__ == "__main__":
    try:
        SnowmapTransformer()
    except rospy.ROSInterruptException: pass

