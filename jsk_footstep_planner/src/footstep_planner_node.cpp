// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_footstep_planner/footstep_planner.h"
#include "jsk_footstep_planner/footstep_graph.h"
////
#include <visualization_msgs/MarkerArray.h>
////
#define DEBUG 0
#if DEBUG
#include <visualization_msgs/MarkerArray.h>
namespace jsk_footstep_planner
{
  extern ros::Publisher pub_debug_marker;
}
#endif
int main(int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  ros::init(argc, argv, "footstep_planner");
  ros::NodeHandle pnh("~");
#if DEBUG
  jsk_footstep_planner::pub_debug_marker = pnh.advertise<visualization_msgs::MarkerArray>("debug_marker_array", 1);
#endif
  jsk_footstep_planner::FootstepPlanner planner(pnh);
  /////
  ros::Publisher pub_line_debug = pnh.advertise<visualization_msgs::MarkerArray>("debug_marker_array", 1);
  ros::Duration sl(2.0);
  sl.sleep();
#if 0
  Eigen::Vector3f p0(0, 0, 0);
  Eigen::Vector3f p1(-1, 3, 0);
  Eigen::Vector3f p2(3, 4, 0);
  Eigen::Vector3f p3(4, 4, 0);
  std::vector <Eigen::Vector3f> pts;
  pts.push_back(p0);
  pts.push_back(p1);
  pts.push_back(p2);
  pts.push_back(p3);
#endif
  Eigen::Vector3f p0(0, 0, 0);
  Eigen::Vector3f p1(0, -1, 0);
  Eigen::Vector3f p2(0, -2, 0);
  Eigen::Vector3f p3(0, -4, 0);
  std::vector <Eigen::Vector3f> pts;
  pts.push_back(p0);
  pts.push_back(p1);
  pts.push_back(p2);
  pts.push_back(p3);
  jsk_recognition_utils::PolyLine la(pts);
  ROS_INFO("length = %f", la.length());

  planner.setHeuristicPathLine(la);

  visualization_msgs::Marker mk;
  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 0;
  la.toMarker(mk);
  mk.header.frame_id = "odom";
  mk.id = 0;
  Eigen::Vector3f q(3, 2, 0);
  Eigen::Vector3f ft;
  double distance_to_goal, alp;
  int idx;
  double dist = la.distanceWithInfo(q, ft, distance_to_goal, idx, alp);

  std::cerr << la << std::endl;

  ROS_INFO("dist = %f, foot = [%f %f %f] (%d on %f), to_goal = %f\n",
           dist, ft[0], ft[1], ft[2], idx, alp,
           distance_to_goal);

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(mk);
  {
    visualization_msgs::Marker pm;
    pm.type = visualization_msgs::Marker::POINTS;
    pm.id = 2;
    pm.header.frame_id = "odom";
    geometry_msgs::Point pp;
    pp.x = q[0];
    pp.y = q[1];
    pp.z = q[2];
    pm.points.push_back(pp);
    pm.scale.x = 0.04;
    pm.scale.y = 0.04;
    pm.color.a = 1;
    pm.color.r = 0;
    pm.color.g = 0;
    pm.color.b = 1;
    ma.markers.push_back(pm);
  }
  {
    visualization_msgs::Marker pm;
    pm.type = visualization_msgs::Marker::ARROW;
    pm.id = 3;
    pm.header.frame_id = "odom";
    geometry_msgs::Point st;
    geometry_msgs::Point ed;
    st.x = q[0];
    st.y = q[1];
    st.z = q[2];
    ed.x = ft[0];
    ed.y = ft[1];
    ed.z = ft[2];
    pm.points.push_back(st);
    pm.points.push_back(ed);
    pm.scale.x = 0.015;
    pm.scale.y = 0.025;
    pm.color.a = 1;
    pm.color.r = 0;
    pm.color.g = 1;
    pm.color.b = 0;
    ma.markers.push_back(pm);
  }
  pub_line_debug.publish(ma);
  /////
  ros::spin();
}
