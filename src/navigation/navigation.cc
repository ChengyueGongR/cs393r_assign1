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
#include "navigation/simple_queue.h"
#include "shared/math/line2d.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

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

Navigation::Navigation(const string& map_file, const double dist, const double curv, ros::NodeHandle* n) :
    point_cloud(),
    initialized(true),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    goal_vertex_id(""),
    runs_since_path_calc(6),
    nav_goal_set_(false) {
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
    map_.Load(map_file);
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

// Helper Function to compute euclidean distance
double ComputeEucDist(const double x, const double y) {
    return sqrt(Sq(x) + Sq(y));
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    point_cloud.clear();
    for (Vector2f point : cloud) {
        point_cloud.push_back(point);
    }
}

Vector2f Navigation::GlobalizePoint(const Vector2f& local_point) {
    float range = ComputeEucDist(local_point.x(), local_point.y());
    float angle_orig = atan2(local_point.y(), local_point.x());
    float angle = robot_angle_ + angle_orig;
    Vector2f global_point(range * cos(angle), range * sin(angle));
    global_point += robot_loc_;
    return global_point;
}

// Uses local message
void Navigation::DrawCar(const Vector2f& local_point, uint32_t color, float angle) {
    Vector2f p1(local_point.x() - 0.5*(h-wheelbase), local_point.y() + 0.5*w + angle);
    Vector2f p2(local_point.x() + 0.5*(h+wheelbase), local_point.y() + 0.5*w);
    Vector2f p3(local_point.x() + 0.5*(h+wheelbase), local_point.y() - 0.5*w);
    Vector2f p4(local_point.x() - 0.5*(h-wheelbase), local_point.y() - 0.5*w);
    visualization::DrawLine(GlobalizePoint(p1), GlobalizePoint(p2), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p2), GlobalizePoint(p3), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p3), GlobalizePoint(p4), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p4), GlobalizePoint(p1), color, local_viz_msg_);
}

void Navigation::Run() {
    if (!initialized)
        return;

    // constants
    const float curv_inc = 0.2;
    const float frontgoal_dist = 2.0;
    float dist_to_frontgoal = frontgoal_dist;

    // relative goal
    Vector2f frontgoal(frontgoal_dist, 0.0);
    Vector2f laser_loc(0.2, 0.0);
    // visuals
    visualization::ClearVisualizationMsg(local_viz_msg_);
    DrawCar(Vector2f(0,0), 0xFF0000, 0.0);

    // evaluate possible paths
    float best_curv = 0;
    float best_score = -std::numeric_limits<float>::max();
    float best_fpl = 0;

    for (float curv = -1; curv <= 1; curv += curv_inc) {
        float fpl = frontgoal_dist;
        float clearance = 0.2;
        Vector2f dest;

        if (abs(curv) < .05) {
            curv = 0;
            // going straight
            for (Vector2f point : point_cloud)
                if ((point.x() >= 0) && (abs(point.y()) <= w)){
                   float fpl_cur = point.x()+laser_loc.x() - (h+wheelbase)*0.5;
		   fpl = std::min(fpl, fpl_cur);
                }
	    for (Vector2f point : point_cloud)
                if (point.x() >= 0 && point.x() <= fpl + (h + wheelbase)*0.5){
                    // if the point is not behind the car and within the path
                    // calculate clearance
		    float fpl_cur = abs(point.y()) - w*0.5;
                    clearance = std::min(clearance, fpl_cur);
		}
            dist_to_frontgoal = ComputeEucDist(abs(fpl - frontgoal.x()), frontgoal.y());
            dest = Vector2f(fpl, 0);
        } else {
            float r = 1.0 / curv;
            Vector2f g(frontgoal.x(), frontgoal.y());
            bool turn_right = false;

            if (r < 0) {
                r = -r;
                turn_right = true;
            }
 
	    fpl = 0.5*3.14*r;
            double r_min = r - 0.5*w;
	    double r_corner = ComputeEucDist(r - 0.5*w, 0.5*(h+wheelbase));
            double r_max = ComputeEucDist(r + 0.5*w, 0.5*(h+wheelbase));

            // Compute FPL
            for (Vector2f point : point_cloud) {
                if (point.x() < 0)
                    continue;

                float point_y = point.y();
                
		if (turn_right) {
                    point_y = -point.y();
                }
                
                double r_point = ComputeEucDist(point.x()+laser_loc.x(), point_y - r);
                double theta_p = atan2(point.x()+laser_loc.x(), r - point_y);

                if (r_point >= r_min && r_point <= r_corner && theta_p > 0) {
                    float curv_dist = 0.0;
                    // hit inside
                    double theta_org = atan2(sqrt(Sq(r_point)-Sq(r_min)),r_min); //acos(r_min, r_point);
		    curv_dist = (theta_p - theta_org) * r;	    

		    if (curv_dist < -.05)
                        continue;
                    fpl = std::min(fpl, curv_dist);
                }
                if (r_point > r_corner && r_point <= r_max && theta_p > 0) {
                    float curv_dist = 0.0;
                    // hit front
                    double theta_org = atan2(0.5*(h+wheelbase), sqrt(Sq(r_point)-Sq(0.5*(h+wheelbase)))); // asin(0.5*(h+wheelbase), r_point);
		    curv_dist = (theta_p - theta_org) * r;	    

		    if (curv_dist < -.05)
                        continue;
                    fpl = std::min(fpl, curv_dist);
                }
            }

            // find clearance of curved path
            for (Vector2f point : point_cloud) {
                if (point.x() < 0)
                    continue;
                float point_y = point.y();

                if (turn_right) {
                    point_y = -1 * point.y();
                }
                
		double r_point = ComputeEucDist(point.x()+laser_loc.x(), point_y - r);
                double theta_p = atan2(point.x()+laser_loc.x(), r - point_y);
                float curv_dist = 0.0;
		if (r_point >= r_min && r_point <= r_corner && theta_p > 0) {
                    // hit inside
                    double theta_org = atan2(sqrt(Sq(r_point)-Sq(r_min)),r_min); //acos(r_min, r_point);
		    curv_dist = (theta_p - theta_org) * r;	    
                }
                if (r_point > r_corner && r_point <= r_max && theta_p > 0) {
                    // hit front
                    double theta_org = atan2(0.5*(h+wheelbase), sqrt(Sq(r_point)-Sq(0.5*(h+wheelbase))));//asin(0.5*(h+wheelbase), r_point);
		    curv_dist = (theta_p - theta_org) * r;	    
                }

                if (curv_dist < fpl && curv_dist >= 0) {
                    float clear_cur = clearance;
                    if (r_point > r) {
                        clear_cur = r_point - r_max;
                    } else if (r_point < r) {
                        clear_cur = r_min - r_point;
                    }
                    clearance = std::min(clearance, clear_cur);
                }
            }

            // make r negative again if flipped
            r = 1.0 / curv;

            float rad = fpl / r;
            float dest_x = r * sin(rad);
            float dest_y = r - r * cos(rad);
            dest = Vector2f(dest_x, dest_y);
            dist_to_frontgoal = ComputeEucDist(dest_x - frontgoal.x(), dest_y - frontgoal.y());
        }

        float w1 = 0.0;
        float w2 = -1.0;
        
        float score = fpl + w1 * clearance + w2 * dist_to_frontgoal;
        
	if (score > best_score) {
	    // Rule Out impossible paths by 1DTOC
            best_score = score;
            best_curv = curv;
            best_fpl = fpl;
        }
        visualization::DrawPathOption(curv, fpl, clearance, local_viz_msg_);
    }

    visualization::DrawPathOption(best_curv, best_fpl, 0, local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
    //if (best_fpl <= 0.01)
    //    return;
    
    double max_v = 1.0;
    drive_msg_.velocity = max_v;
    drive_msg_.curvature = 0;//best_curv;
    drive_pub_.publish(drive_msg_);

  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
