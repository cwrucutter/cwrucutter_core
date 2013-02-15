#!/usr/bin/env python
import roslib; roslib.load_manifest('cutter_amcl')

from nav_msgs.srv import *
from nav_msgs.msg import *
import rospy
import tf

class LandmarkServer:
  def __init__(self):
    # Start the node
    rospy.init_node('landmark_map_server')

    landmarkfile = rospy.get_param('~file_landmarks','landmarks.txt')
    self.file_landmarks = roslib.packages.get_pkg_dir('cutter_amcl')+'/'+landmarkfile    
    rospy.loginfo('Landmark file will be:\n '+ self.file_landmarks)
    
    self.pts_landmarks = []
    self.map = OccupancyGrid()
    
    r = rospy.Rate(0.1)
    if self.CreateMap() == 1:
      self.map_srv = rospy.Service('static_map', GetMap, self.HandleMapService)
      self.map_pub = rospy.Publisher('landmark_map', OccupancyGrid)
      print "Ready to serve a landmark map."
      while not rospy.is_shutdown():
        print "Publishing map"
        self.map_pub.publish(self.map)
        r.sleep()
    else:
      rospy.logerr('Map creation was not successful. Leaving Node')
      
  
  def ReadFiles(self):
    # ReadFiles function reads the survey file and the snowmap file to extract coordinates
    f_lm = open(self.file_landmarks,'r')
    
    # Extract csv
    for line in f_lm:                  # iterates through each line in the file
        res = map(float,line.split(','))   # parses the csv
        self.pts_landmarks.append(res)        # appends to the list
    f_lm.close()
    
    # Print points
    print "\nLandmark Points:"
    print self.pts_landmarks

  def CreateMap(self):
    # CreateMap function creates the nav_msgs/OccupancyGrid based on the stored data
    self.ReadFiles();
    print "\nCreating Map:"
    pts_lm = self.pts_landmarks
    
    width = 10  #meters
    height = 20 #meters
    res = 0.1   #meters
    origin = [-3,-4]
    tht = 0
    rot = tf.transformations.quaternion_from_euler(0, 0, tht)
   
    size = len(pts_lm)
    if size > 1:
      # 1. Populate Map metadata
      self.map.info.resolution = 0.10;
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
        print int(map_coord[0]*self.map.info.width + map_coord[1])
        print len(self.map.data)
        self.map.data[int(map_coord[0] + map_coord[1]*self.map.info.width)] = 100
      
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
