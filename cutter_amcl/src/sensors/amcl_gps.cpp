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
// Author: EJ Kreinar
// Date: 15 Nov 2012
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include "amcl_gps.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLGps::AMCLGps() : AMCLSensor()
{
  this->time = 0.0;

  return;
}

void 
AMCLGps::SetModelLeverarm(double sigma_gps)
{
  this->model_type = GPS_MODEL_LEVERARM;
  this->sigma_gps = sigma_gps;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the gps sensor model
bool AMCLGps::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the gps sensor model 
  //   - If we add other GPS models, add more if-statements to select model
  if(this->model_type == GPS_MODEL_LEVERARM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) GpsModel, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) GpsModel, data);

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLGps::GpsModel(AMCLGpsData *data, pf_sample_set_t* set)
{
  AMCLGps *self;
  int i;
  double z, pz;
  double p;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLGps*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    pose = sample->pose;

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
    
    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}
