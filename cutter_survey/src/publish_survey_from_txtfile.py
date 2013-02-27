#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# EJ Kreinar
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
from geometry_msgs.msg import Point
from cutter_msgs.msg import Survey

class SnowmapTransformer:
    def __init__(self):
        # Start the node
        rospy.init_node('recorded_survey_publisher')
        surveyfile = rospy.get_param('~filename','survey.txt')
        topic_out = rospy.get_param('~topic_out','/cwru/survey')
        self.file_survey = roslib.packages.get_pkg_dir('cutter_survey')+'/survey/'+surveyfile
        
        rospy.loginfo('Survey file will be:\n '+ self.file_survey)
        
        self.pts_survey = []  # List to contain surveyed points
        self.surveyPts = Survey()
        
        self.ReadFiles()
        
        self.pub = rospy.Publisher(topic_out, Survey)

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.pub.publish(self.surveyPts)
            rate.sleep()

        
    
    def ReadFiles(self):
        # ReadFiles function reads the survey file and the snowmap file to extract coordinates
        f_survey = open(self.file_survey,'r')
        
        # Extract csv
        for line in f_survey:                  # iterates through each line in the file
            res = map(float,line.split(','))   # parses the csv
            self.pts_survey.append(res)        # appends to the list
            pt = Point()
            pt.x = res[0]
            pt.y = res[1]
            pt.z = res[2]
            self.surveyPts.points.append(pt)
        f_survey.close()
        
        # Print points
        print "\nSurveyed Points:"
        print self.pts_survey
        

if __name__ == "__main__":
    try:
        SnowmapTransformer()
    except rospy.ROSInterruptException: pass

