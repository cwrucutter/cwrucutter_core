#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Charles Hart
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
from cutter_msgs.msg import StatusUpdate
from cutter_msgs.msg import Switches

class cRioTranslator():

  def __init__(self):
      rospy.init_node('crio_status_translator')
      rospy.loginfo('Initializing cRIO status translator...')
      topic_switches = rospy.get_param('~topic_switches','/cwru/switches')
      topic_in = rospy.get_param('~topic_in','/cwru/status')
      rospy.loginfo('Publishing to: '+topic_switches)
      rospy.loginfo('Subscribing to: '+topic_in)
      
      self.called = False	# Don't publish before input is ready

      self.pub = rospy.Publisher(topic_switches, Switches)
      rospy.Subscriber(topic_in, StatusUpdate, self.callback)
      
      while not rospy.is_shutdown():
        rospy.sleep(1)
         
  def callback(self,data):
      self.called = True 	# Now it's okay to publish
      s = Switches()
      s.switchA = (data.switches & 0b0001)/0b0001
      s.switchB = (data.switches & 0b0010)/0b0010
      s.switchC = (data.switches & 0b0100)/0b0100
      s.switchD = (data.switches & 0b1000)/0b1000
      s.mainEstop   = (data.safety & 0b000001)/0b000001
      s.remoteEstop = (data.safety & 0b000010)/0b000010
      s.wheelRelay  = (data.safety & 0b000100)/0b000100
      s.bladeRelay  = (data.safety & 0b001000)/0b001000
      s.heartBeat   = (data.safety & 0b010000)/0b010000
      s.driveEnable = (data.safety & 0b100000)/0b100000
      self.pub.publish(s)

if __name__ == "__main__":
    try:
        cRioTranslator()
    except rospy.ROSInterruptException: pass

