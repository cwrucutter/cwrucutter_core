#!/usr/bin/env python
import roslib; roslib.load_manifest('cutter_amcl')

from nav_msgs.srv import *
from nav_msgs.msg import *
from math import fabs
from cutter_msgs.msg import Survey
from geometry_msgs.msg import Point
import rospy
import tf

class LandmarkServer:
  def __init__(self):
    # Start the node
    rospy.init_node('landmark_map_server')

    topic_in = rospy.get_param('~topic_in','/cwru/survey')
    
    self.pts_landmarks = []
    self.map = OccupancyGrid()
    self.sub = rospy.Subscriber(topic_in, Survey, self.SurveyCallback)
    
    # Give 2 seconds to call callbacks and get settled
    r = rospy.Rate(1)
    r.sleep()
    r.sleep()
    
    r = rospy.Rate(0.1)
    self.map_srv = rospy.Service('static_map', GetMap, self.HandleMapService)
    self.map_pub = rospy.Publisher('landmark_map', OccupancyGrid)
    print "Ready to serve a landmark map."
    while not rospy.is_shutdown():
      print "Publishing map"
      self.map_pub.publish(self.map)
      r.sleep()
  
  def SurveyCallback(self, data):
    print "Points Received"
    anyNew = False
    for newpt in data.points:
      pt = [newpt.x, newpt.y, newpt.z]
      new = True
      for oldpt in self.pts_landmarks:
        if fabs(oldpt[0] - pt[0]) < 0.001 and fabs(oldpt[1] - pt[1]) < 0.001:
          new = False
          continue
      if new:
        anyNew = True
        print pt
        self.pts_landmarks.append(pt)
    print self.pts_landmarks
    
    if anyNew:
      self.CreateMap()
    

  def CreateMap(self):
    # CreateMap function creates the nav_msgs/OccupancyGrid based on the stored data
    print "\nCreating NEW Map:"
    pts_lm = self.pts_landmarks
    size = len(pts_lm)
    
    width = 30.0  #meters
    height = 30.0 #meters
    res = 0.1   #meters
    origin = [0,0]
    
    
    if size > 1:
      # First: Determine the center of the surveyed pts...
      #  this will be the center of the map 
      xsum = 0; ysum = 0;
      for pt in pts_lm:
        xsum = xsum + pt[0]
        ysum = ysum + pt[1]
      xmean = xsum/size
      ymean = ysum/size
      print xmean, ymean
      origin = [xmean-width/2, ymean-height/2]
      print origin
      tht = 0
      rot = tf.transformations.quaternion_from_euler(0, 0, tht)
      
      # 1. Populate Map metadata
      self.map.info.resolution = res;
      self.map.info.width = width / res;
      self.map.info.height = height / res;
      self.map.info.origin.position.x = origin[0]
      self.map.info.origin.position.y = origin[1]
      self.map.info.origin.orientation.x = rot[0]
      self.map.info.origin.orientation.y = rot[1]
      self.map.info.origin.orientation.z = rot[2]
      self.map.info.origin.orientation.w = rot[3]
      self.map.data = [0 for i in range(int(self.map.info.width * self.map.info.height))]
      
      # 2. Populate data based on landmark location
      for lm in pts_lm:
        print "Processing Landmark: "
        print lm
        map_coord = tuple([lm[i]-origin[i] for i in range(len(origin))])
        print map_coord
        map_coord = tuple([x/res for x in map_coord])
        print map_coord
        print int(int(map_coord[0]) + int(map_coord[1])*self.map.info.width)
        self.map.data[int((map_coord[0]) + int(map_coord[1])*self.map.info.width)] = 100
        #self.map.data[int(map_coord[0]*self.map.info.height + map_coord[1])] = 100
      
      # 3. Populate Header
      
      # 4. Print some stuff
          
    else:
      rospy.logerr('Number of landmark points ('+str(len(pts_survey))
                 + ') must be greater than 0')
      return 0
    
    return 1
    
  def HandleMapService(self, req):
    print "Serving a map"
    return GetMapResponse(self.map)

if __name__ == "__main__":
    try:
        LandmarkServer()
    except rospy.ROSInterruptException: pass




"""
def handle_landmark_map(req):
    print "Serving a map"
    
    return AddTwoIntsResponse(req.a + req.b)

def landmark_map_server():
    rospy.init_node('landmark_map_server')
    s = rospy.Service('static_map', GetMap, handle_landmark_map)
    print "Ready to serve a landmark map."
    rospy.spin()

if __name__ == "__main__":
    landmark_map_server()
"""
