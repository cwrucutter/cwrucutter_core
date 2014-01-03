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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from gps_common.msg import GPSStatus
from geometry_msgs.msg import TwistStamped


import serial, string, math, struct, binascii

# Parses my custom binary output from the Arduimu. 
# Stupid Arduimu.

#Calculate the CRC for supplied data (bytearray)
def CRC(data):
    chk = 0;
    for b in data:
        chk = chk ^ b;
    return chk


class ArduimuParser:
    """
        Parses a binary Arduimu IMU Message
    """
    def __init__(self):
        """ Initialize the Parser """
        self.hdr_msgID   = 0;

        self.MSG_GPS = 1;
        self.MSG_IMU = 2;

    def VerifyChecksum(self, data, chk):
        """ Verify the Checksum, return bool """
        rcvChk = struct.unpack('<H', chk)  # Received Checksum
        genChk = CRC(bytearray(data))  # Generated Checksum
        return (rcvChk == genChk)      # Compare
    
    def ParseHeader(self, data):
        """ Read the Header, return bool if we have the correct messageID and Length """
        header = struct.unpack('<B',data);
        rospy.loginfo('Header')
        rospy.loginfo(header)
        self.hdr_msgID = header[0]
        return True

    def ParseGPS(self, data, gpsMsg, velMsg, statMsg):
        """ Parse a Serial message, populate gpsMsg
            data: string
            gpsMsg: ROS message gps
        """
        gps_msg = struct.unpack('<llhhhLBB',data)
        lat, lon, alt, speed, trkAngle, time, nsats, status = gps_msg
        rospy.loginfo(data)
        rospy.loginfo(gps_msg)

        # Populate the GPS message
        gpsMsg.latitude  = lat/10000000.0;
        gpsMsg.longitude = lon/10000000.0;
        gpsMsg.altitude  = alt/100.0;
        #TODO: Replace the covariance
        gpsMsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        velMsg.twist.linear.x = float(speed)*0.514444444444*math.sin(math.radians(float(trkAngle)))
        velMsg.twist.linear.y = float(speed)*0.514444444444*math.cos(math.radians(float(trkAngle)))
        
        statMsg.satellites_used = nsats;
        statMsg.status = status;

        return True
        
    def ParseIMU(self, data, imuMsg):
        """ Parse a Serial message, populate imuMsg
            data: string
            imuMsg: ROS message imu
        """
        imu_msg = struct.unpack('<ffffff',data)
        gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = imu_msg
        print(bytearray(data))
        rospy.loginfo(imu_msg)
        
        # Populate the Angular Velocity
        imuMsg.angular_velocity.x = gyro_x
        imuMsg.angular_velocity.y = gyro_y
        imuMsg.angular_velocity.z = gyro_z
        #TODO: Populate the covariance

        # Populate the Linear Acceleration
        imuMsg.linear_acceleration.x = accel_x
        imuMsg.linear_acceleration.y = accel_y
        imuMsg.linear_acceleration.z = accel_z
        #TODO: Populate the covariance

        #TODO: Populate the orientation/magentometer
    
        return True


if __name__ == "__main__":
    #ROS init
    rospy.loginfo("1")
    rospy.init_node('arduimu_driver')
    imuPub = rospy.Publisher('arduimu/imu',Imu)
    gpsPub = rospy.Publisher('arduimu/gps_fix',NavSatFix)
    velPub = rospy.Publisher('arduimu/gps_vel',TwistStamped)
    statPub = rospy.Publisher('arduimu/gps_status',GPSStatus);
    #Init Imu port
    arduPort = rospy.get_param('~port','/dev/ttyUSB0')
    arduRate = rospy.get_param('~baud',57600)

    rospy.loginfo("2")
    imuData = Imu()
    navData = NavSatFix()
    gpsVel = TwistStamped()
    gpsStat = GPSStatus()
    imuData.header.frame_id = "base_arduimu"
    navData.header.frame_id = "base_arduimu"
    gpsVel.header.frame_id  = "base_arduimu"
    gpsStat.header.frame_id = "base_arduimu"

    rospy.loginfo("3")
    parser = ArduimuParser()
    
    try:
        rospy.loginfo("4")
        dataIn = serial.Serial(port=arduPort,baudrate=arduRate,timeout=.02)
        #Read in data
        sync0 = '\x00'; sync1 = '\x00';
        while not rospy.is_shutdown():
            # READ UNTIL SYNC
            data  = dataIn.read(1)
            sync1 = sync0; sync0 = data;
            sync  = sync1+sync0;
            match = '\x55\xAA'
            if sync != match:
                rospy.loginfo('Header didnt match')
                continue
            else:
                rospy.loginfo("Beginning new message")

            # READ HEADER
            header = dataIn.read(1)
            if (not parser.ParseHeader(header)):
                rospy.logwarn("Packet Failed: Header could not be parsed")
                continue
            
            # READ MESSAGE
            if parser.hdr_msgID == parser.MSG_IMU:
                msg = dataIn.read(24)
            elif parser.hdr_msgID == parser.MSG_GPS:
                msg = dataIn.read(20)
            else:
                rospy.logwarn("Arduimu message ID not recognized. Supported messages: Gps, Imu")
            
            # READ CRC
            chk = dataIn.read(1)
            #if (not parser.VerifyChecksum(msg,chk)):    
            #    rospy.logwarn("Packet Failed: CRC Did not Match")
            #    continue
            
            # PARSE MESSAGE
            timeNow = rospy.get_rostime()
            if parser.hdr_msgID == parser.MSG_IMU:
                #HS_SERIAL_IMU_MSG message
                imuData.header.stamp = timeNow
                parser.ParseIMU(msg,imuData)
            
                # Publish imuData
                imuPub.publish(imuData)
            elif parser.hdr_msgID == parser.MSG_GPS:
                navData.header.stamp = timeNow
                gpsVel.header.stamp = timeNow
                gpsStat.header.stamp = timeNow
                parser.ParseGPS(msg,navData, gpsVel, gpsStat)
            
                # Publish navData and status
                gpsPub.publish(navData)
                velPub.publish(gpsVel)
                statPub.publish(gpsStat)
            else:
                rospy.logwarn("Arduimu message ID not recognized. Supported messages: Gps, Imu")
            
    except rospy.ROSInterruptException:
        IMU.close() #Close GPS serial port
