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
    state_loc(0, 0),
    state_angle(0),
    odom_initialized_(false)
    map_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
    
  if(map_initialized_ && odom_initialized_) {
    *loc = state_loc;
    *angle = state_angle;
  }
  return ; 
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
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

void GetRasterMatrix(const vector<Vector2f>& loc,
                     const float& step,
                     const float sensor_noise,
                     MatrixXf* raster_ptr) {

  MatrixXf& raster_matrix = *raster_ptr;

  // loop    
  for (int i = 0; i < ((int)raster_matrix.size()); i++) {
    for (int j = 0; j < ((int)raster_matrix[0].size()); j++) {
      raster_matrix(i, j) = -10;
      
      // denote that we pass the location info here
      for(const auto& p: loc) {
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
