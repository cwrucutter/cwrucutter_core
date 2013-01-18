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
from cutter_msgs.msg import Beacon 

import serial, string, math, time, calendar, struct


#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Calculates the checksum for our custom message type
# --> XOR Of all bytes 
def CalculateChkSum(data):
    CRC = 0
    for idx in range(len(data)):
        CRC = CRC ^ data[idx]
    return CRC

class BeaconParser:
    """
        Parses a binary Message from the beacon
    """
    def __init__(self):
        """ Initialize the Parser """
        self.msg_id      = 0;
        self.num_beacons = 0;
        
        self.RANGE = 1;

    def VerifyChecksum(self, data, CRC):
        """ Verify the Checksum, return bool """
        chk = struct.unpack('<L', CRC)
        checksum = CalculateChkSum(bytearray(data))
        return (chk[0] == checksum)
    
    def ParseHeader(self, data):
        """ Read the Header, return bool """
        header = struct.unpack('<BB',data);
        self.msg_id      = header[0]
        self.num_beacons = header[1]
        return (len(data) == 2)

    def ParseRange(self, data, beaconData):
        """ Parse Data from the beacon, store result in beaconMsg
            data: string (length 2 bytes)
            beaconMsg: ROS message for holding beacon data 
        """
        beaconData = struct.unpack('<h',data)
        rospy.loginfo(beaconData)

        return True


if __name__ == "__main__":
    #ROS init
    rospy.init_node('ultrasound_beacon_driver')
    beaconPub = rospy.Publisher('cwru/beacon', Beacon)
    #Init GPS port
    beaconPort = rospy.get_param('~port','/dev/ttyUSB0')
    beaconRate = rospy.get_param('~baud',57600)

    beaconMsg = Beacon()

    parser = BeaconParser()
    
    try:
        SRL = serial.Serial(port=beaconPort, baudrate=beaconRate, timeout=.01)
        #Read in GPS data
        sync0 = '\x00'; sync1 = '\x00'; sync2 = '\x00';
        while not rospy.is_shutdown():
            # READ UNTIL SYNC
            data  = SRL.read(1)
            sync1 = sync0; sync0 = data;
            sync  = sync1+sync0;
            match = '\x55\xAA'
            if sync != match:
                rospy.loginfo("Sync message failed")
                continue
            else:
                rospy.loginfo("Beginning new message")

            # READ HEADER
            header = SRL.read(2)
            if (not parser.ParseHeader(header)):
                rospy.logwarn("Packet Failed: Unexpected header size")
                continue

            # READ MESSAGE
            msg = []
            for i in range(parser.num_beacons):
                data = SRL.read(2) #Each beacon provides 2 bytes of data
                if (len(msg) != 2):
                    rospy.loginfo("Packet Failed: Message length unexpected")
                    continue
                msg.append(data)

            # READ CRC
            chk = GPS.read(1)
            if (not parser.VerifyChecksum(header+msg,chk)):
                rospy.logwarn("Packet Failed: CRC Did not Match")
                continue

            # PARSE MESSAGE
            timeNow = rospy.get_rostime()
            if parser.msg_id == parser.RANGE:
                #All beacon messages have the same header:
                #  - Published at the same time
                #  - Published from the same receiver (relative to the robot origin)
                beaconMsg.header.stamp = timeNow
                beaconMsg.header.frame_id = 'base_ranger_1'
                for idx in range(len(msg)):
                    # Parse the message for a single beacon
                    parser.ParseRange(msg[idx], beaconMsg.range)
                    # Set the frame where the range originated
                    beaconMsg.beacon_frame = 'beacon_'+str(idx+1)
                    # Publish the message
                    beaconPub.publish(beaconMsg)

            else:
                rospy.logwarn("Beacon message id not recognized")

    except rospy.ROSInterruptException:
        SRL.close() #Close GPS serial port
