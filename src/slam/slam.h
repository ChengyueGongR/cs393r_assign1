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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

// GTSam includes.
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
  Eigen::Vector2f state_loc;
  float state_angle;

  // map or base_link referenced point cloud 
  std::vector<Eigen::Vector2f> point_cloud;
};
  
struct DeltaS {
  Eigen::Vector2f delta_loc;
  float delta_angle;
};
 
class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  
  // init
  Eigen::Vector2f prev_state_loc_;
  float prev_state_angle_;
  Eigen::Vector2f state_loc_;
  float state_angle_;
  bool map_initialized_;
  
  std::vector<Pose> map_pose_;
 
  float const trans_thres_ = 0.75;
  float const angle_thres_ = M_PI/6;
 
  float const raster_height_ = 8.5;
  float const raster_width_ = 5.5; 
  float const raster_step_ = 0.05;
  
  Eigen::MatrixXf raster_matrix_{int(2*raster_height_/raster_step_)+1, int(2*raster_width_/raster_step_)+1}; // maintain a matrix and update it 
 
  float const sensor_sigma_ = 0.25; // need to tune
  
  std::vector<DeltaS> delta_s_;
  
};
  
std::vector<Eigen::Vector2f> GetPointCloud(const std::vector<float>& ranges,
                                 const float angle_min,
                                 const float angle_max);
 
void GetRasterMatrix(const std::vector<Eigen::Vector2f>& pc,
                      const float& step,
                      const float sensor_noise,
                      Eigen::MatrixXf* raster_ptr);

float RasterWeighting(const Eigen::MatrixXf& raster_matrix,
                      const float raster_step,
                      const std::vector<Eigen::Vector2f>& point_cloud);
}  // namespace slam

#endif   // SRC_SLAM_H_
