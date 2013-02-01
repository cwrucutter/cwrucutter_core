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


#Calculates the checksum for our custom message type
# --> XOR Of all bytes 
def CalculateChkSum(data):
    CRC = 0
    for idx in range(len(data)):
        CRC = CRC ^ int(data[idx])
    return CRC

class BeaconParser:
    """
        Parses a binary Message from the beacon
    """
    def __init__(self):
        """ Initialize the Parser """
        self.msg_id      = 0;
        self.last_msg_id = 0;
        self.num_beacon  = 0;
        self.beaconRange = 0.0;
        
        self.RANGE_CONST = 1;

    def VerifyChecksum(self, data, CRC):
        """ Verify the Checksum, return bool """
        chk = struct.unpack('<B', CRC)
        checksum = CalculateChkSum(bytearray(data))
        return (chk[0] == checksum)
    
    def ParseHeader(self, data):
        """ Read the Header, return bool """
        if len(data) != 2:
            return False
        self.last_msg_id = self.msg_id;
        header = struct.unpack('<BB',data);
        self.msg_id      = header[0]
        self.num_beacon  = header[1]
        return True

    def ParseMessage(self, data):
        """ Parse Data from the beacon, store result in member variables
            data: string (length 2 bytes)
        """
        if len(data) != 2:
            return False
        output = struct.unpack('>H',data)
        self.beaconRange = output[0]
        return True
    
    def PopulateMsg(self, beaconMsg):
        """ Populate the ROS beaconMsg
            beaconMsg: ROS message (cutter_msgs/Beacon)
        """
        # Set the frame where the range originated (beacon number)
        beaconMsg.beacon_frame = 'beacon_'+str(parser.num_beacon)
        # Set the range and bearing
        beaconMsg.range = float(self.beaconRange)
        beaconMsg.bearing = 0.0  # not included here (range-only beacons)
        
      

if __name__ == "__main__":
    #ROS init
    rospy.init_node('ultrasound_beacon_driver')
    beaconPub = rospy.Publisher('cwru/beacon', Beacon)
    #Init Serial port
    beaconPort = rospy.get_param('~port','/dev/ttyUSB0')
    beaconRate = rospy.get_param('~baud',57600)

    beaconMsg = Beacon()
    parser = BeaconParser()
    
    try:
        SRL = serial.Serial(port=beaconPort, baudrate=beaconRate, timeout=5)
        #Read in GPS data
        sync0 = '\x00'; sync1 = '\x00'; sync2 = '\x00';
        while not rospy.is_shutdown():
            # READ UNTIL SYNC
            data  = SRL.read(1)
            sync1 = sync0; sync0 = data;
            sync  = sync1+sync0;
            match = '\x55\xAA'
            if sync != match:
                continue
            else:
                rospy.loginfo("Beginning new message")

            # READ HEADER
            header = SRL.read(2)
            if (not parser.ParseHeader(header)):
                rospy.logwarn("Packet Failed: Error reading header")
                continue
            if (parser.msg_id != parser.last_msg_id+1):
                rospy.logwarn("Message ID unexpected.")

            # READ MESSAGE
            msg = SRL.read(2)
            if (not parser.ParseMessage(msg)):
                rospy.logwarn("Packet Failed: Error reading message data");
                continue
                
            # READ CRC
            chk = SRL.read(1)
            if (not parser.VerifyChecksum(header+msg,chk)):
                rospy.logwarn("Packet Failed: CRC Did not Match")
                continue
                
            print parser.beaconRange
            print hex(parser.beaconRange)

            # PARSE MESSAGE
            if (not parser.num_beacon > 5):
                timeNow = rospy.get_rostime()
                # Set the header
                beaconMsg.header.stamp = timeNow
                beaconMsg.header.frame_id = 'base_ranger_1'
                # Populate the message based on the parsed data
                parser.PopulateMsg(beaconMsg)
                # Publish the message
                beaconPub.publish(beaconMsg)
            else:
                rospy.logwarn("Beacon Error! Not publishing data")
                continue

    except rospy.ROSInterruptException:
        SRL.close() #Close serial port
