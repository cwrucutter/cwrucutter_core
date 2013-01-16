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
// Desc: Beacon sensor model for AMCL
// Author: EJ Kreinar
// Date: 15 Jan 2013
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_BEACON_H
#define AMCL_BEACON_H

#include "amcl_sensor.h"
//#include "../map/map.h"
#include "../pf/pf_pdf.h"

namespace amcl
{

typedef enum
{
  BEACON_MODEL_RANGE_ONLY,
  BEACON_MODEL_BEARING_ONLY,
  BEACON_MODEL_RANGE_BEARING
} beacon_model_t;

// Beacon sensor data
class AMCLBeaconData : public AMCLSensorData
{
  public:
    AMCLBeaconData () {};
    virtual ~AMCLBeaconData() {};
  public: double range;
  public: double angle;
};


// Beacon sensor model
class AMCLBeacon : public AMCLSensor
{
  // Default constructor
  public: AMCLBeacon();

  public: void SetModelRangeOnly(double sigma_range);
  
  public: void SetModelBearingOnly(double sigma_bearing);
  
  public: void SetModelRangeBearing(double sigma_range,
                                    double sigma_bearing);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Set the Beacons's pose
  public: void SetBeaconPose(pf_vector_t& beacon_pose) 
          {this->beacon_pose = beacon_pose;}
  
  // Set the Receiver pose
  public: void SetReceiverPose(pf_vector_t& receiver_pose) 
          {this->receiver_pose = receiver_pose;}

  // Determine the probability for the given pose
  private: static double RangeModel(AMCLBeaconData *data, 
                                          pf_sample_set_t* set);
  private: static double BearingModel(AMCLBeaconData *data, 
                                          pf_sample_set_t* set);
  private: static double RangeBearingModel(AMCLBeaconData *data, 
                                          pf_sample_set_t* set);


  private: beacon_model_t model_type;

  // Current data timestamp
  private: double time;

  // Beacon location (beacon_pose is in the GLOBAL frame)
  private: pf_vector_t beacon_pose;
  
  // Receiver offset relative to robot origin (receiver_pose is in the LOCAL frame)
  private: pf_vector_t receiver_pose;
  
  // Beacon model params
  //
  // Stddev of Gaussian model for gps values hits.
  private: double sigma_range;
  private: double sigma_bearing;
};


}

#endif
