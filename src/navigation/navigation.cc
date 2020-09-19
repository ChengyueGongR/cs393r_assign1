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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================


#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::max;
using std::min;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}


float Navigation::Run1DTOC(float x_now,
                           float x_target,
                           float v_now,
                           float max_speed,
                           float a_max,
                           float d_max,
                           float dt) const {
  // static FILE* fid = nullptr;
  // const bool test_mode = false;
  const bool test_toc = false; 
  const bool test_obstacle = false;
  const bool test_avoidance = false;
  const bool test_mode =
      test_toc || test_obstacle || test_avoidance;
  const float dist_left = x_target - x_now;
  float velocity_cmd = 0;
  const float speed = fabs(v_now);
  const float dv_a = dt * a_max;
  const float dv_d = dt * d_max;
  float accel_stopping_dist =
      (speed + 0.5 * dv_a) * dt +
      Sq(speed + dv_a) / (2.0 * d_max);
  float cruise_stopping_dist =
      speed * dt +
      Sq(speed) / (2.0 * d_max);
  char phase = '?';
  if (dist_left >  0) {
    if (speed < max_speed && accel_stopping_dist < dist_left) {
      // Acceleration possible.
      phase = 'A';
      velocity_cmd = min<float>(max_speed, speed + dv_a);
    } else if (cruise_stopping_dist < dist_left) {
      // Must maintain speed, cruise phase.
      phase = 'C';
      velocity_cmd = speed;
    } else {
      // Must decelerate.
      phase = 'D';
      velocity_cmd = max<float>(0, speed - dv_d);
    }
  } else if (speed > 0.0f) {
    phase = 'X';
    velocity_cmd = max<float>(0, speed - dv_d);
  }
  if (test_mode) {
    printf("%c x:%f dist_left:%f a_dist:%f c_dist:%f v:%f cmd:%f\n",
           phase,
           x_now,
           dist_left,
           accel_stopping_dist,
           cruise_stopping_dist,
           v_now,
           velocity_cmd);
  }
  return velocity_cmd;
}

void Navigation::Run() {
    
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::DrawCross(Vector2f(2, 0), 0.2, 0xFF0000, local_viz_msg_);
  viz_pub_.publish(local_viz_msg_);

  /*
  // move it to head file
  float free_path_length = 30.0;
  float obstacle_margin = 0.15;
  float max_speed = 0.5;
  float max_accel = 1.0;
  float max_decel = 1.0;
  float dt = 0.025;

  const float speed = 1.0;
  const float dist_left = max<float>(0.0f, free_path_length - obstacle_margin);
  */
  drive_msg_.curvature = 1.0;
  drive_msg_.velocity = 10.0f; // Run1DTOC(0, dist_left, speed, max_speed, max_accel, max_decel, dt);
  drive_pub_.publish(drive_msg_);
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
