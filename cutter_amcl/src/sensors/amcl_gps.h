/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: GPS sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_gps.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_GPS_H
#define AMCL_GPS_H

#include "amcl_sensor.h"
//#include "../map/map.h"
#include "../pf/pf_pdf.h"

namespace amcl
{

typedef enum
{
  GPS_MODEL_LEVERARM
} gps_model_t;

// Laser sensor data
class AMCLGpsData : public AMCLSensorData
{
  public:
    AMCLGpsData () {};
    virtual ~AMCLGpsData() {};
  public: double x;
  public: double y;
};


// Laseretric sensor model
class AMCLGps : public AMCLSensor
{
  // Default constructor
  public: AMCLGps();

  public: void SetModelLeverarm(double sigma_gps);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Set the GPS's pose after construction
  public: void SetGpsPose(pf_vector_t& gps_pose) 
          {this->gps_pose = gps_pose;}

  // Determine the probability for the given pose
  private: static double GpsModel(AMCLGpsData *data, 
                                   pf_sample_set_t* set);

  private: gps_model_t model_type;

  // Current data timestamp
  private: double time;

  // Gps offset relative to robot
  private: pf_vector_t gps_pose;
  
  // Gps model params
  //
  // Stddev of Gaussian model for gps values hits.
  private: double sigma_gps;
};


}

#endif
