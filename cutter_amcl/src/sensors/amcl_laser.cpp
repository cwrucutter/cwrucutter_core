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
// Desc: AMCL laser routines
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

#include "amcl_laser.h"

//radius of the beacon cones
#define RADIUS_OF_CONE 0.135

//offset distance between 
//#define LIDAR_OFFSET 0.85

//----------------------------------------------
//set up the laser pointer
//#define LASER_ARRAY_POINTER
//----------------------------------------------

//Cone x,y
/*
 4  3
 5  2
 0  1
*/
#define CONE_NUM_MAX	6
#define CONE_X	0
#define CONE_Y	1
double cone_position[CONE_NUM_MAX][2]=
{
	{RADIUS_OF_CONE,-3.0+RADIUS_OF_CONE},		//Cone 0
	{4.0-RADIUS_OF_CONE,-3.0+RADIUS_OF_CONE},	//Cone 1
	{4.0-RADIUS_OF_CONE,5.0},					//Cone 2
	{4.0-RADIUS_OF_CONE,15.0-RADIUS_OF_CONE},	//Cone 3
	{RADIUS_OF_CONE,15.0-RADIUS_OF_CONE},		//Cone 4
	{RADIUS_OF_CONE,5.0}						//Cone 5
};

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t* map) : AMCLSensor()
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

void 
AMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
AMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);

  return true;
}

double Cone2LaserAngle(double robot_x,double robot_y,double robot_psi,double cone_x,double cone_y)
{
	double scan_angle;
	double lidar_center_x,lidar_center_y;
	
	//lidar_center_x=robot_x-LIDAR_OFFSET*sin(robot_psi);
	//lidar_center_y=robot_y-LIDAR_OFFSET*cos(robot_psi);
	
	//scan_angle=atan2((lidar_center_y-cone_y),(lidar_center_x-cone_x))+robot_psi;
	scan_angle=atan2((robot_y-cone_y),(robot_x-cone_x))+robot_psi;
	
	return scan_angle;
}

//get distance between Cone and robot
double Cone2Distance(double robot_x,double robot_y,double cone_x,double cone_y)
{
	double distance;
	double lidar_center_x,lidar_center_y;
	
	//lidar_center_x=robot_x-LIDAR_OFFSET*sin(robot_psi);
	//lidar_center_y=robot_y-LIDAR_OFFSET*cos(robot_psi);
	
	//distance=fabs(sqrt(lidar_center_x-cone_x)+sqrt(lidar_center_y-cone_y));
	
  double xoff, yoff;
  xoff = robot_x - cone_x;
  yoff = robot_y - cone_y;
  distance = sqrt( xoff*xoff + yoff*yoff );
	
	return distance;
}

//estimate the distance difference between laser measurement and expecting cone positions
double LaserConeError(double distance_expect,double distance_laser_real)
{
	double distance_error;
	
	distance_error=fabs(distance_expect-distance_laser_real)/distance_laser_real;
	
	return distance_error;
	
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;
    
    unsigned char temp_local_i;
    unsigned char cone_counter;
    double temp_angle,temp_error,temp_total;
    int index;

    temp_error=0;
    temp_total=0;
    cone_counter=0;

    for(temp_local_i=0;temp_local_i<CONE_NUM_MAX;temp_local_i++)
    {
	    //get the expecting scan angle
	    temp_angle= Cone2LaserAngle(pose.v[0],pose.v[1],pose.v[2],cone_position[temp_local_i][CONE_X],cone_position[temp_local_i][CONE_Y]);
	
	    //check if it can be seen by lider
	    if(temp_angle>0||temp_angle<M_PI)
	    {
		    //then check the distance error(expecting data Vs. real laser data)
		    map_range = Cone2Distance(pose.v[0], pose.v[1], cone_position[temp_local_i][CONE_X],cone_position[temp_local_i][CONE_Y]);
		    		    
        step = (data->range_count - 1) / (self->max_beams - 1);
        index = round(temp_angle) * 180 / M_PI;
		    obs_range = data->ranges[index][0];
		    
		    //temp_error=LaserConeError(map_range,LASER_ARRAY_POINTER[temp_angle]);	
		    //temp_total+=temp_error;
		    //cone_counter++;
		    
		    // Part 1: good, but noisy, hit
        z = obs_range - map_range;
        pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

        // Part 2: short reading from unexpected obstacle (e.g., a person)
        if(z < 0)
          pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

        // Part 3: Failure to detect obstacle, reported as max-range
        if(obs_range == data->range_max)
          pz += self->z_max * 1.0;

        // Part 4: Random measurements
        if(obs_range < data->range_max)
          pz += self->z_rand * 1.0/data->range_max;

        // TODO: outlier rejection for short readings

        assert(pz <= 1.0);
        assert(pz >= 0.0);
        //      p *= pz;
        // here we have an ad-hoc weighting scheme for combining beam probs
        // works well, though...
        p += pz*pz*pz;
	    }
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0/data->range_max;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}
