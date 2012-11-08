/* Copyright (c) 2012, EJ Kreinar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ********************************************************************************
 * GPS_conversion.h
 *   Provides utitility function for GPS conversion (LLA to ENU) 
 *
 ********************************************************************************/
#ifndef GPS_CONVERSION
#define GPS_CONVERSION

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>

namespace GPS_conversion{

const double a  = 6378137;      // meters, Semi Major Axis
const double b  = 6356752.3142; // meters, Semi Minor Axis
const double e2 = 1-pow(b/a,2); // first eccentricity squared


static void LLA2ENU(const sensor_msgs::NavSatFix &gps, const sensor_msgs::NavSatFix &ref, geometry_msgs::Pose &enu)
{
  // Based off the paper by S.P. Drake: http://www.dsto.defence.gov.au/publications/2443/DSTO-TN-0432.pdf

  // Location of reference point in radians
  double phi = ref.latitude*M_PI/180;
  double lam = ref.longitude*M_PI/180;
  double h   = ref.altitude;

  // Location of gps point in radians
  double dphi = gps.latitude*M_PI/180 - phi;
  double dlam = gps.longitude*M_PI/180 - lam;
  double dh   = gps.altitude - h;
  
  // Constants
  double tmp = sqrt(1-e2*pow(sin(phi),2));
  double cl = cos(lam);
  double sl = sin(lam);
  double cp = cos(phi);
  double sp = sin(phi);

  // Transformations
  double de = (a/tmp+h)*cp*dlam - (a*(1-e2)/(pow(tmp,3))+h)*sp*dphi*dlam + cp*dlam*dh;
  double dn = (a*(1-e2)/pow(tmp,3) + h)*dphi + 1.5*cp*sp*a*e2*pow(dphi,2) + pow(sp,2)*dh*dphi + 0.5*sp*cp*(a/tmp+h)*pow(dlam,2);
  double du = dh - 0.5*(a-1.5*a*e2*pow(cp,2)+0.5*a*e2+h)*pow(dphi,2) - 0.5*pow(cp,2)*(a/tmp-h)*pow(dlam,2);

  enu.position.x = de;
  enu.position.y = dn;
  enu.position.z = du;
  
  return;
}

}


#endif
