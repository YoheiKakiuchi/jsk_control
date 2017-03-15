// -*- mode: c++ -*-

#ifndef JSK_FOOTSTEP_PLANNER_GRID_PATH_PLANNER_H_
#define JSK_FOOTSTEP_PLANNER_GRID_PATH_PLANNER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// ros
#include <sensor_msgs/PointCloud2.h>
//#include <dynamic_reconfigure/server.h>
#include <jsk_rviz_plugins/OverlayText.h>
//#include <jsk_footstep_msgs/FootstepArray.h>
//#include <jsk_footstep_msgs/PlanFootstepsAction.h>
//#include <jsk_footstep_planner/FootstepPlannerConfig.h>
#include <jsk_footstep_planner/CollisionBoundingBoxInfo.h>

// grid path planning
#include "jsk_footstep_planner/grid_astar_solver.h"
#include "jsk_footstep_planner/grid_graph.h"

namespace jsk_footstep_planner
{
  enum GridPlanningStatus {
    NONE,
    OK,
    WARNING,
    ERROR,
  };
  /**
   * @brief
   * Actionlib server for footstep planning
   */
  class GridPathPlanner
  {
  public:
    GridPathPlanner(ros::NodeHandle& nh);

  protected:
    virtual void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void planCB(const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal);

    virtual bool collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res);
    // buildGraph is not thread safe, it is responsible for caller to take care
    // of mutex
    virtual void buildGraph();

    virtual void configCallback(Config &config, uint32_t level);

    virtual void profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph);

    virtual void publishPointCloud(
      const pcl::PointCloud<pcl::PointNormal>& cloud,
      ros::Publisher& pub,
      const std_msgs::Header& header);

    virtual void publishText(ros::Publisher& pub,
                             const std::string& text,
                             PlanningStatus status);

    virtual boll updateCost(GridState::Ptr ptr);
    //solver.openListToPointCloud(open_list_cloud);
    //solver.closeListToPointCloud(close_list_cloud);

    boost::mutex mutex_;
    actionlib::SimpleActionServer<jsk_footstep_msgs::PlanFootstepsAction> as_;
    jsk_footstep_msgs::PlanFootstepsResult result_;

    ros::Publisher pub_close_list_;
    ros::Publisher pub_open_list_;
    ros::Publisher pub_text_;

    ros::Subscriber sub_pointcloud_model_; // plane for stepping
    ros::Subscriber sub_obstacle_model_;   // collision detection with environment

    pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_model_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model_;

    PerceptionGridMap::Ptr gridmap_;
    PerceptionGridGraph::Ptr graph_;

    Eigen::Vector3f collision_bbox_size_;
    Eigen::Affine3f collision_bbox_offset_;
    Eigen::Vector3f map_offset_;

    // Parameters
    double map_resolution_;
    double collision_circle_radius_;
    double collision_circle_min_height_;
    double collision_circle_max_height_;

bool use_obstacle_model_;
bool use_pointcloud_model_;
  private:
  };
}

#endif
