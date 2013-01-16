/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
// Desc: AMCL GPS routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include "amcl_beacon.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLBeacon::AMCLBeacon() : AMCLSensor()
{
  this->time = 0.0;

  return;
}

void AMCLBeacon::SetModelRangeOnly(double sigma_range)
{
  this->sigma_range = sigma_range;
}

void AMCLBeacon::SetModelBearingOnly(double sigma_bearing)
{
  this->sigma_bearing = sigma_bearing;
}


void AMCLBeacon::SetModelRangeBearing(double sigma_range, double sigma_bearing)
{
  this->sigma_range = sigma_range;
  this->sigma_bearing = sigma_bearing;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the gps sensor model
bool AMCLBeacon::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the beacon sensor model 
  if(this->model_type == BEACON_MODEL_RANGE_ONLY)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) RangeModel, data);
  else if(this->model_type == BEACON_MODEL_BEARING_ONLY)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BearingModel, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) RangeBearingModel, data);

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLBeacon::RangeModel(AMCLBeaconData *data, pf_sample_set_t* set)
{
  AMCLBeacon *self;
  int i;
  double z, pz;
  double p;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLBeacon*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    pose = sample->pose;
    /*
    // Take account of the gps pose relative to the robot
    pose = pf_vector_coord_add(self->gps_pose, pose);

    p = 1.0;
    pz = 0.0;
    
    // Gaussian model
    // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
    //      ... but its okay, because it will get normalized anyway
    z = data->x - pose.v[0];
    p *= exp(-(z * z) / (2 * self->sigma_gps * self->sigma_gps));
    
    z = data->y - pose.v[1];
    p *= exp(-(z * z) / (2 * self->sigma_gps * self->sigma_gps));
    
    // Ad hoc method- add pz's together, if the sample weight gets too small
    */
    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLBeacon::BearingModel(AMCLBeaconData *data, pf_sample_set_t* set)
{
  // NOT CURRENTLY IMPLEMENTED
  
  AMCLBeacon *self;
  int i;
  double z, pz;
  double p;
  double total_weight;
  pf_sample_t *sample;

  self = (AMCLBeacon*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    // dont change the sample weight
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLBeacon::RangeBearingModel(AMCLBeaconData *data, pf_sample_set_t* set)
{
  // NOT CURRENTLY IMPLEMENTED
  
  AMCLBeacon *self;
  int i;
  double z, pz;
  double p;
  double total_weight;
  pf_sample_t *sample;

  self = (AMCLBeacon*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    // dont change the sample weight
    total_weight += sample->weight;
  }

  return(total_weight);
}
