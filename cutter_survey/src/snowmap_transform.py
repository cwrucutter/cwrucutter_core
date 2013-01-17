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
        surveyfile = rospy.get_param('~file_survey','survey.txt')
        snowmapfile = rospy.get_param('~file_snowmap','snowmap.txt')
        self.file_survey = roslib.packages.get_pkg_dir('cutter_survey')+'/survey/'+surveyfile
        self.file_snowmap = roslib.packages.get_pkg_dir('cutter_survey')+'/survey/'+snowmapfile
        
        rospy.loginfo('Survey file will be:\n '+ self.file_survey)
        rospy.loginfo('Snowmap file will be:\n '+ self.file_snowmap)
        
        self.pts_survey = []  # List to contain surveyed points
        self.pts_snowmap = [] # List to contain corresponding snowmap points
        self.translate = (0.0, 0.0, 0.0) 
        self.rotate = (0.0, 0.0, 0.0, 1.0)
        
        self.ReadFiles()
        
        if self.CreateTransform() == 1:
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
        
    
    def ReadFiles(self):
        # ReadFiles function reads the survey file and the snowmap file to extract coordinates
        f_survey = open(self.file_survey,'r')
        f_snowmap = open(self.file_snowmap,'r')
        
        # Extract csv
        for line in f_survey:                  # iterates through each line in the file
            res = map(float,line.split(','))   # parses the csv
            self.pts_survey.append(res)        # appends to the list
        for line in f_snowmap:
            res = map(float,line.split(','))
            self.pts_snowmap.append(res)
        f_survey.close()
        f_snowmap.close()
        
        # Print points
        print "\nSurveyed Points:"
        print self.pts_survey
        print "\nSnowmap Points:"
        print self.pts_snowmap
        
    def AngleDistOffset(self, pt1, pt2):
        #print pt1, pt2
        thtA = math.atan2(pt2[1]-pt1[1], pt2[0]-pt1[0])
        distA = math.sqrt( (pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2 )
        return thtA, distA

    def CreateTransform(self):
        # CreateTransform function creates the transform based on the stored data
        print "\nCreating Transform:"
        pts_survey = self.pts_survey
        pts_snowmap = self.pts_snowmap
       
        size = len(pts_survey)
        if size == len(pts_snowmap) and size > 1:
            # 1. Find the Angle Offset
            tht = []
            dist = []
            for i in range(size-1):
                for j in range(size-1-i):
                    #print i,size-j-1
                    thtA, distA = self.AngleDistOffset(pts_survey[i], pts_survey[size-j-1])
                    thtB, distB = self.AngleDistOffset(pts_snowmap[i], pts_snowmap[size-j-1])                    
                    #print distA, distB, distA-distB
                    tht.append(thtA-thtB)
                    dist.append(distA-distB)
            thtOff = float(sum(tht)/len(tht)) if len(tht) > 0 else float('nan')
            distMax = max(dist)
            #print dist
            #if distMax > 1:
            #    rospy.logerr("Cannot complete transformation. " 
            #                 + "Surveyed and Predicted points do not align. "
            #                 + "Distance offset: " + str(distMax) )
            #    return 0
            
            # 2. Find the Translation offset
            xvals = []
            yvals = []
            for i in range(size):
                ptA = pts_survey[i]
                ptB = pts_snowmap[i]
                
                xoff = ptA[0] - ptB[0]*math.cos(thtOff) + ptB[1]*math.sin(thtOff)
                yoff = ptA[1] - ptB[0]*math.sin(thtOff) - ptB[1]*math.cos(thtOff)
                xvals.append(xoff)
                yvals.append(yoff)
            xOff = float(sum(xvals)/len(xvals)) if len(xvals) > 0 else float('nan')
            yOff = float(sum(yvals)/len(yvals)) if len(yvals) > 0 else float('nan')
            
            # 3. Save the Transformation Values!!
            self.translate = (xOff, yOff, 0.0)
            self.rotate = tf.transformations.quaternion_from_euler(0, 0, thtOff)
            
            # 4. Print a bunch of stuff
            print "\n - Possible Angles:"
            print tht
            print " - Max Distance Offset:"
            print distMax
            print dist
            print " - Average Angle:"
            print thtOff
            print "\n - Possible Translations:"
            print xvals
            print yvals
            print " - Average Translation:"
            print xOff, yOff
            print ""
            
        else:
            rospy.logerr('Number of surveyed points ('+str(len(pts_survey))
                       + ') is not equal to the number of snowmap points ('
                       + str(len(pts_snowmap)) + ').' )
            return 0
        
        return 1

if __name__ == "__main__":
    try:
        SnowmapTransformer()
    except rospy.ROSInterruptException: pass

