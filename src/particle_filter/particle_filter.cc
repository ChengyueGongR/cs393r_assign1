//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

#define NUM_PARTICLES 50
DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  /*scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }
  //printf("map size:%lu", map_.lines.size());
  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
  }*/
  scan.resize(num_ranges);
  printf("map size:%lu", map_.lines.size());
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
    float angle_cur = angle_min + i*(angle_max-angle_min)/(num_ranges-1);
    // To global frame
    angle_cur = angle_cur + angle;
    // The laser scan from this angle
    line2f line_cur(range_min*cos(angle_cur), range_min*sin(angle_cur), range_max*cos(angle_cur),range_max*sin(angle_cur));
    //loop over map
    float length_cur = range_max;
    scan[i].x() = range_max*cos(angle_cur);
    scan[i].y() = range_max*sin(angle_cur);
    for (size_t j = 0; j < map_.lines.size(); ++j){
      const line2f map_line = map_.lines[i];
      bool intersects = map_line.Intersects(line_cur);
      if (intersects){
        Vector2f intersection_point; // Return variable
        intersects = map_line.Intersection(line_cur, &intersection_point);
        float length = sqrt(pow(intersection_point.x()-range_min*cos(angle_cur),2)+pow(intersection_point.y()-range_min*sin(angle_cur),2));
	if (length<length_cur){
	  scan[i].x() = intersection_point.x();
	  scan[i].y() = intersection_point.y();
	}
      }
    } 
  }

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  int num_ranges = ranges.size();
  vector<Vector2f> scan;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, num_ranges, range_min, range_max, angle_min, angle_max, &scan);
  float sigma_ob = 0.05;
  float gamma =0.9;
  p_ptr->weight = 0.0;
  for (size_t i = 0; i < ranges.size(); ++i){
      float range = ranges[i];
      float range_predicted = sqrt(pow(scan[i].x()-p_ptr->loc.x(),2)+pow(scan[i].y()-p_ptr->loc.y(),2));
      p_ptr->weight += gamma * (-0.5) * (pow(range-range_predicted,2)/pow(sigma_ob,2));
  }
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  //float x = rng_.UniformRandom(0, 1);
  //printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //       x);
  
  //Pre-process to avoid numerical issues
  float log_weight_min = 100.0;
  for (Particle& p : particles_){
    if (p.weight<log_weight_min){
      log_weight_min = p.weight;
    }
  }
  float sum_weight = 0.0;
  for (Particle& p : particles_){
    p.weight = p.weight - log_weight_min;
    p.weight = exp(p.weight);
    sum_weight += p.weight;
  }
  for (Particle& p : particles_){
    p.weight = p.weight/sum_weight;
  }
  
  // Resample
  vector<Particle> new_particles;
  for (int i=0; i<NUM_PARTICLES;++i){
    Particle new_particle;
    float rand_num = rng_.UniformRandom(0, 1);
    float sum_weight_cur = 0.0;
    for (Particle& p : particles_){
      if ((sum_weight_cur<=rand_num) && (sum_weight_cur+p.weight>=rand_num)){
        new_particle = p;
	break;
      }
      sum_weight_cur += p.weight;
    }
    new_particles.push_back(new_particle);
  }
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  //printf("range_min:%f, range_max:%f, angle min:%f, angle max:%f", range_min, range_max,angle_min, angle_max);
  for (Particle& p : particles_){
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  }
  Resample();
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  //float x = rng_.Gaussian(0.0, 2.0);
  //printf("Odometry: Random number drawn from Gaussian distribution with 0 mean and "
  //       "standard deviation of 2 : %f\n", x);
  // Conver odom to map, then add noise to the particles
  if (odom_initialized_){
    printf("ODOM STARTED");
    prev_odom_loc_.x() = odom_loc.x();
    prev_odom_loc_.y() = odom_loc.y();
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = false;
  }
  float k1 = 0.1; // t on t
  float k2 = 0.1; // r on t
  float k3 = 0.1; // t on r
  float k4 = 0.1; // r on r

  float delta_x = odom_loc.x() - prev_odom_loc_.x();
  float delta_y = odom_loc.y() - prev_odom_loc_.y();
  float delta_angle = odom_angle - prev_odom_angle_;
  

  for (Particle& p : particles_){
    // Convert ODOM to MAP frame
    float delta_x_base = cos(-prev_odom_angle_)*delta_x - sin(-prev_odom_angle_)*delta_y;
    float delta_y_base = sin(-prev_odom_angle_)*delta_x + cos(-prev_odom_angle_)*delta_y;
    float delta_x_hat = cos(p.angle)*delta_x_base - sin(p.angle)*delta_y_base;
    float delta_y_hat = sin(p.angle)*delta_x_base + cos(p.angle)*delta_y_base;
    float delta_angle_hat = delta_angle;
    // Add noise 
    float variance_t = k1 * sqrt(pow(delta_x_hat,2)+pow(delta_y_hat,2)) + k2 * fabs(delta_angle_hat);
    float variance_r = k3 * sqrt(pow(delta_x_hat,2)+pow(delta_y_hat,2)) + k4 * fabs(delta_angle_hat);
    float ep_x = rng_.Gaussian(0.0, variance_t);
    float ep_y = rng_.Gaussian(0.0, variance_t);
    float ep_angle = rng_.Gaussian(0.0, variance_r);
    p.loc.x() = p.loc.x() + delta_x_hat + ep_x;
    p.loc.y() = p.loc.y() + delta_y_hat + ep_y;
    p.angle = p.angle + delta_angle_hat + ep_angle;
  }
  prev_odom_loc_.x() = odom_loc.x();
  prev_odom_loc_.y() = odom_loc.y();
  prev_odom_angle_ = odom_angle;
    
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  printf("Initilized, loc: (%f, %f), angle: %f\n", loc.x(), loc.y(), angle);
  for (int i=1;i<=NUM_PARTICLES;i++) {
    struct Particle p;
    float x = rng_.Gaussian(0.0, 0.1);
    p.loc.x() = loc.x() + x;
    float y = rng_.Gaussian(0.0, 0.1);
    p.loc.y() = loc.y() + y;
    float ang = rng_.Gaussian(0.0, 0.05);
    p.angle = angle + ang;
    p.weight = 1.0;
    printf("%d: particle location:(%f, %f), rotation: %f, weight: %f\n", i, p.loc.x(), p.loc.y(), p.angle, p.weight);
    particles_.push_back(p);
  }
  odom_initialized_ = true;
  map_.Load("maps/" + map_file + ".txt");
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  Vector2f loc_mean = Vector2f(0, 0);
  float angle_mean = 0.0;
  for (Particle p : particles_){
    loc_mean.x() = loc_mean.x() + p.loc.x();
    loc_mean.y() = loc_mean.y() + p.loc.y();
    angle_mean = angle_mean + p.angle;
  }
  loc.x() = loc_mean.x()/NUM_PARTICLES;
  loc.y() = loc_mean.y()/NUM_PARTICLES;
  angle = angle_mean/NUM_PARTICLES;
  //printf("loc: %f, %f, angle: %f\n", loc.x(), loc.y(), angle);
}

}  // namespace particle_filter
