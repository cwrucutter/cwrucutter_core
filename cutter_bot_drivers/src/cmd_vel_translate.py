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
roslib.load_manifest('cutter_bot_drivers')
import rospy
from cutter_msgs.msg import VelCmd
from geometry_msgs.msg import Twist
from cutter_msgs.msg import Switches  

class VelTranslator():

      def __init__(self):
          rospy.init_node('cmd_vel_translator')
          topic_out = rospy.get_param('~topic_out','/cwru/cmd_vel')
          topic_in  = rospy.get_param('~topic_in','/cmd_vel')
          rospy.loginfo('Publishing to: '+topic_out)
          rospy.loginfo('Subscribing to: '+topic_in)

          self.pub = rospy.Publisher(topic_out,VelCmd)
          rospy.Subscriber(topic_in, Twist, self.callback)
          rospy.Subscriber("cwru/switches", Switches, self.callerback)       
          
          self.switchA = True
          self.driveEnable = False
          self.vel_called = False
          
          while not rospy.is_shutdown():              
            if not self.vel_called:
              cmd_vel = VelCmd()
              cmd_vel.linear = 0
              cmd_vel.angular = 0 
              self.pub.publish(cmd_vel)
            self.vel_called = False
            rospy.sleep(.5)
             
      def callback(self,data): 
          self.vel_called = True
          cmd_vel = VelCmd()
          cmd_vel.linear  = data.linear.x*1000.0
          cmd_vel.angular = data.angular.z*1000.0
          if cmd_vel.linear > 2000:
             cmd_vel.linear = 2000
          if cmd_vel.linear < -2000:
             cmd_vel.linear = -2000
          if not self.switchA and self.driveEnable:
             self.pub.publish(cmd_vel)
          else:
             self.vel_called = False
             rospy.logwarn("robot cannot accept vel comands now")
          
      def callerback(self,dater):
          self.switches = Switches()
          self.switchA = dater.switchA
          self.driveEnable = dater.driveEnable
      
      
      
if __name__ == "__main__":
    try:
        VelTranslator()
    except rospy.ROSInterruptException: pass

