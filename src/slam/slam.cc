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
\file    slam.cc
\brief   SLAM Starter Code
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
#include "slam.h"

#include "vector_map/vector_map.h"

using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement SLAM

// Milestone 2 will be implemented here.

namespace slam {

// config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

SLAM::SLAM() :
  prev_odom_loc_(0, 0),
  prev_odom_angle_(0),
  prev_state_loc_(0, 0),
  prev_state_angle_(0),
  state_loc_(0, 0),
  state_angle_(0),
  odom_initialized_(false)
  map_initialized_(false) {
    // Construct voxel cube
    int const loc_samples_ = 10;
    float const loc_std_dev = 0.25; 
    int const angle_samples_ = 20; 
    float const angle_std_dev = 0.25; 
    for(auto a=-angle_samples_; a <= angle_samples_; a++) {
      float relative_angle_sample = a*angle_std_dev/angle_samples_;

      for(auto x=-loc_samples_; x <= loc_samples_; x++) {
        for(auto y = -loc_samples_; y <= loc_samples_; y++) {
          Vector2f relative_loc_sample( ix*loc_std_dev/loc_samples_,
                                        iy*loc_std_dev/loc_samples_ );
          
          Voxel p0{relative_loc_sample, relative_angle_sample};
          
          voxel_cube_.push_back(p0);
        }
      }
    }
  }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
    
  if(map_initialized_ && odom_initialized_) {
    *loc = state_loc;
    *angle = state_angle;
  }
  return ; 
}    
    
vector<Vector2f> GetPointCloud(const vector<float>& ranges,
                                  const float angle_min,
                                  const float angle_max) {
  // copy from navi and patile_filter
  float const bias = 0.2;

  vector<Vector2f> point_cloud;
  point_cloud.reserve( ranges.size() );

  float const angle_step = (angle_max - angle_min)/ranges.size();
  for(auto i=0; i<ranges.size(); i++)
  {
    double const theta = angle_min + angle_step * i; 
    x_i = ranges[i] * cos(theta) + 0.2;
    y_i = ranges[i] * sin(theta_i);

    point_cloud.push_back(Vector2f(x_i, y_i));
  }
  
  return point_cloud;
}
    
float RasterWeighting(const Eigen::MatrixXf& raster_matrix,
                         const float raster_step,
                         const std::vector<Eigen::Vector2f>& point_cloud) {
  float likelihood = 0;
  int i, j; // index

  // read all point 
  for(auto& p: point_cloud) {
    // Check if the point is within the rasters dimensions
    if( fabs(p.x()) < raster_step*(raster_matrix.size()-1)/2 &&
        fabs(p.y()) < raster_step*(raster_matrix[0].size()-1)/2 ) {
      i = p.x()/raster_step;
      j = p.y()/raster_step;

      likelihood += raster_matrix(i + (raster.rows()-1) / 2, j + (raster.cols()-1) / 2);
    }
  }

  return likelihood;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
    
  // check the implementation, here is still some bugs
  if (!map_initialized_ && odom_initialized_ ) {
    map_pose_.clear(); // clear the log
    
    // init the cloud
    Pose pc0{state_loc_, state_angle_, 
                GetPointCloud(ranges, angle_min, angle_max)};
    map_pose_.push_back(pc0);
      
    // set flag
    map_initialized_ = true;
    prev_state_loc_ = state_loc_;
    prev_state_angle_ = state_angle_;

    GetRasterMatrix(p0.point_cloud, raster_step_, sensor_sigma_, &raster_matrix_);
    return ; 
  }
  
  if (odom_initialized_ && map_initialized_ && 
      ((prev_state_loc_ - state_loc_).norm() > trans_thres_ ||
      abs(prev_state_angle_ - state_angle_) > angle_thres_)) {
      
    GetRasterMatrix(map_pose_.back().point_cloud, raster_step_, sensor_sigma_, &raster_matrix_);
    vector<Vector2f> pc = GetPointCloud(ranges, angle_min, angle_max);
    
    Vector2f const delta_loc_ = state_loc_ - prev_state_loc_ ; // mle
    float const delta_angle_ = state_angle_ - prev_state_angle_;
      
    // set value
    prev_state_loc_ = state_loc_;
    prev_state_angle_ = state_angle_;
    
    Vector2f delta_loc(0, 0);
    float delta_angle = 0;
    float likelihood = -100;

    for(const auto& v: voxel_cube_) {
      // fixing ... 
      // first: transform point cloud
      vector<Vector2f> transformed_pc;
      transformed_pc.reserve(pc.size());
      const Rotation2Df rot(delta_angle_ + v.delta_angle);
      for(auto& p: pc) {
        transformed_pc.push_back(delta_loc_ + v.delta_loc + rot*p);
      }
      
      float const raster_likelihood = RasterWeighting(raster_matrix,
                                                      raster_step_,
                                                      transformed_pc);
      if (likelihood < raster_likelihood) { 
        likelihood = raster_likelihood;
        delta_loc = delta_loc_ + v.delta_loc;
        delta_angle = delta_angle_ + v.delta_angle;
      }
    }
    Pose node{map_pose_.back().state_loc + delta_loc, 
              map_pose_scan_.back().state_angle + delta_angle, 
              loc};

    map_pose_.push_back(mode);

    return;
  }
  
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  
  // copy from particle filter
  float delta_loc = odom_loc - prev_odom_loc_;
  float delta_angle = odom_angle - prev_odom_angle_;
    
    // fixed: here it is negative
  const Rotation2Df bl_rotation(-prev_odom_angle_);   
  // transform
  Vector2f delta_T_bl = bl_rotation * delta_loc;  
  double const delta_angle_bl = delta_angle;
  
  Rotation2Df map_rotation(state_angle_);
  state_loc_ += map_rotation*delta_T_bl;
  state_angle_ += delta_angle_bl;

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  return;
}

void GetRasterMatrix(const vector<Vector2f>& pc,
                     const float& step,
                     const float sensor_noise,
                     MatrixXf* raster_ptr) {

  MatrixXf& raster_matrix = *raster_ptr;

  // loop    
  for (int i = 0; i < ((int)raster_matrix.size()); i++) {
    for (int j = 0; j < ((int)raster_matrix[0].size()); j++) {
      raster_matrix(i, j) = -10;
      
      // denote that we pass the location info here
      for(const auto& p: pc) {
        Vector2f temp((i - (int)raster_matrix.size())*step, (j - (int)raster_matrix[0].size())*step);
        float const prob = (-0.5 * (temp-p).norm() * (temp-p).norm() / (sensor_noise * sensor_noise));
        if (prob > raster_matrix(i, j)) {
          raster_matrix(i, j) = prob;
        }
      }
    }
  }

  return;
}

    
vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
