#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, EJ Kreinar
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
import sys
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

class Surveyor:
    def __init__(self):
        # Start the node
        rospy.init_node('gps_surveyor')
        topic_in  = rospy.get_param('~topic_in','/gps_pose')
        filename = rospy.get_param('~filename','surveyed.txt')
        self.file = roslib.packages.get_pkg_dir('cutter_localization')+'/survey/'+filename
        
        rospy.loginfo('Subscribing to: '+topic_in)
        rospy.loginfo('Output file will be: '+ self.file)
        
        # Initialize
        self.coords = (0.0, 0.0, 0.0)
        
        # Start the ROS stuff
        rospy.Subscriber(topic_in, PoseStamped, self.GPSCallback)
        rospy.Service('survey_gps_point', Empty, self.SurveyServer)
        rospy.spin()
        
    def GPSCallback(self, data):
        # GPS Calback: Called when a new gps point arrives
        self.coords = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
        sys.stdout.write("Current GPS point: (%f, %f, %f) \r" % (data.pose.position.x, data.pose.position.y, data.pose.position.z))
        sys.stdout.flush()

    def SurveyServer(self, req):
        # Survey Server: Called to record the point
        rospy.loginfo("Surveying... Point: %s", ('%f, %f, %f' % self.coords) )
        with open(self.file,'a') as f:
            write_data = f.write(str('%f, %f, %f\n' % self.coords))
        return []

if __name__ == "__main__":
    try:
        Surveyor()
    except rospy.ROSInterruptException: pass

