// -*- mode: c++ -*-

#include "jsk_footstep_planner/footstep_planner.h"

namespace jsk_footstep_planner
{
  GridPathPlanner::GridPathPlanner(ros::NodeHandle& nh):
    as_(nh, nh.getNamespace(),
        boost::bind(&GridPathPlanner::planCB, this, _1), false)
  {
    pub_text_ = nh.advertise<jsk_rviz_plugins::OverlayText> ("text", 1, true);
    pub_close_list_ = nh.advertise<sensor_msgs::PointCloud2> ("close_list", 1, true);
    pub_open_list_  = nh.advertise<sensor_msgs::PointCloud2> ("open_list", 1, true);

    srv_collision_bounding_box_info_ = nh.advertiseService(
      "collision_bounding_box_info", &GridPathPlanner::collisionBoundingBoxInfoService, this);

    buildGraph(); //

    sub_pointcloud_model_ = nh.subscribe("pointcloud_model", 1, &GridPathPlanner::pointcloudCallback, this);
    sub_obstacle_model_   = nh.subscribe("obstacle_model", 1, &GridPathPlanner::obstacleCallback, this);

    std::vector<double> collision_bbox_size, collision_bbox_offset;
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_size", collision_bbox_size)) {
      collision_bbox_size_[0] = collision_bbox_size[0];
      collision_bbox_size_[1] = collision_bbox_size[1];
      collision_bbox_size_[2] = collision_bbox_size[2];
    }
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_offset", collision_bbox_offset)) {
      collision_bbox_offset_ = Eigen::Affine3f::Identity() * Eigen::Translation3f(collision_bbox_offset[0],
                                                                                  collision_bbox_offset[1],
                                                                                  collision_bbox_offset[2]);
    }
    as_.start();
  }

  void GridPathPlanner::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("obstacle model is updated");
    obstacle_model_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *obstacle_model_);
    // obstacle_model_frame_id_ = msg->header.frame_id; // check frame_id

    if (gridmap_ && use_obstacle_model_) {
      gridmap_->setObstacleModel(obstacle_model_);
    }
  }

  void GridPathPlanner::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("pointcloud model is updated");
    pointcloud_model_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pointcloud_model_);
    //pointcloud_model_frame_id_ = msg->header.frame_id; // check frame_id
    if (gridmap_ && use_pointcloud_model_) {
      gridmap_->setPointCloudModel(pointcloud_model_);
    }
  }

  bool GridPathPlanner::collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    res.box_dimensions.x = collision_bbox_size_[0];
    res.box_dimensions.y = collision_bbox_size_[1];
    res.box_dimensions.z = collision_bbox_size_[2];
    tf::poseEigenToMsg(collision_bbox_offset_, res.box_offset);
    return true;
  }

  void GridPathPlanner::publishText(ros::Publisher& pub,
                                    const std::string& text,
                                    GridPlanningStatus status)
  {
    std_msgs::ColorRGBA ok_color;
    ok_color.r = 0.3568627450980392;
    ok_color.g = 0.7529411764705882;
    ok_color.b = 0.8705882352941177;
    ok_color.a = 1.0;
    std_msgs::ColorRGBA warn_color;
    warn_color.r = 0.9411764705882353;
    warn_color.g = 0.6784313725490196;
    warn_color.b = 0.3058823529411765;
    warn_color.a = 1.0;
    std_msgs::ColorRGBA error_color;
    error_color.r = 0.8509803921568627;
    error_color.g = 0.3254901960784314;
    error_color.b = 0.30980392156862746;
    error_color.a = 1.0;
    std_msgs::ColorRGBA color;
    if (status == OK) {
      color = ok_color;
    }
    else if (status == WARNING) {
      color = warn_color;
    }
    else if (status == ERROR) {
      color = error_color;
    }
    jsk_rviz_plugins::OverlayText msg;
    msg.text = text;
    msg.width = 1000;
    msg.height = 1000;
    msg.top = 10;
    msg.left = 10;
    msg.bg_color.a = 0.0;
    msg.fg_color = color;
    msg.text_size = 24;
    pub.publish(msg);
  }

  void GridPathPlanner::planCB(const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // latest_header_ = goal->goal_footstep.header;
    ROS_INFO("planCB");

    // check frame_id sanity
    std::string goal_frame_id = goal->initial_footstep.header.frame_id;
#if 0
    if (use_pointcloud_model_) {
      // check perception cloud header
      if (goal_frame_id != pointcloud_model_frame_id_) {
        ROS_ERROR("frame_id of goal and pointcloud do not match. goal: %s, pointcloud: %s.",
                      goal_frame_id.c_str(), pointcloud_model_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
    if (use_obstacle_model_) {
      // check perception cloud header
      if (goal_frame_id != obstacle_model_frame_id_) {
        ROS_ERROR("frame_id of goal and obstacle pointcloud do not match. goal: %s, obstacle: %s.",
                      goal_frame_id.c_str(), obstacle_model_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
#endif

    result_.result = ros_path;
    as_.setSucceeded(result_);

    pcl::PointCloud<pcl::PointXYZRGB> close_list_cloud, open_list_cloud;
    //solver.openListToPointCloud(open_list_cloud);
    //solver.closeListToPointCloud(close_list_cloud);
    publishPointCloud(close_list_cloud, pub_close_list_, goal->goal_footstep.header);
    publishPointCloud(open_list_cloud,  pub_open_list_,  goal->goal_footstep.header);
#if 0
    publishText(pub_text_,
                (boost::format("Took %f sec\nPerception took %f sec\nPlanning took %f sec\n%lu path\nopen list: %lu\nclose list:%lu")
                 % planning_duration 
                 % graph_->getPerceptionDuration().toSec()
                 % (planning_duration - graph_->getPerceptionDuration().toSec())
                 % path.size()
                 % open_list_cloud.points.size()
                 % close_list_cloud.points.size()).str(),
                OK);
    ROS_INFO_STREAM("use_obstacle_model: " << graph_->useObstacleModel());
#endif
  }

  void GridPathPlanner::publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    ros::Publisher& pub,
    const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header = header;
    pub.publish(ros_cloud);
  }

  void GridPathPlanner::profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph)
  {
    if (as_.isPreemptRequested()) {
      solver.cancelSolve();
      ROS_WARN("cancelled!");
    }
    // ROS_INFO("open list: %lu", solver.getOpenList().size());
    // ROS_INFO("close list: %lu", solver.getCloseList().size());
    publishText(pub_text_,
                (boost::format("open_list: %lu\nclose list:%lu")
                 % (solver.getOpenList().size()) % (solver.getCloseList().size())).str(),
                OK);
    if (rich_profiling_) {
      pcl::PointCloud<pcl::PointNormal> close_list_cloud, open_list_cloud;
      solver.openListToPointCloud(open_list_cloud);
      solver.closeListToPointCloud(close_list_cloud);
      publishPointCloud(close_list_cloud, pub_close_list_, latest_header_);
      publishPointCloud(open_list_cloud, pub_open_list_, latest_header_);
    }
  }
  
  double GridPathPlanner::stepCostHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStepCost(node, graph, heuristic_first_rotation_weight_,
                                     heuristic_second_rotation_weight_);
  }

  double GridPathPlanner::zeroHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicZero(node, graph);
  }

  double GridPathPlanner::straightHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStraight(node, graph);
  }

  double GridPathPlanner::straightRotationHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStraightRotation(node, graph);
  }

  double GridPathPlanner::followPathLineHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicFollowPathLine(node, graph);
  }

  /**
     format is
       successors:
         - x: 0
           y: 0
           theta: 0
         - x: 0
           y: 0
           theta: 0
         ...
   */
  bool GridPathPlanner::readSuccessors(ros::NodeHandle& nh)
  {
    successors_.clear();
    if (!nh.hasParam("successors")) {
      ROS_FATAL("no successors are specified");
      return false;
    }
    // read default translation from right foot to left foot
    double default_x   = 0.0;
    double default_y   = 0.0;
    double default_theta = 0.0;
    if (nh.hasParam("default_lfoot_to_rfoot_offset")) {
      std::vector<double> default_offset;
      if (jsk_topic_tools::readVectorParameter(nh, "default_lfoot_to_rfoot_offset", default_offset)) {
        default_x =     default_offset[0];
        default_y =     default_offset[1];
        default_theta = default_offset[2];
      }
    }
    // read successors
    XmlRpc::XmlRpcValue successors_xml;
    nh.param("successors", successors_xml, successors_xml);
    if (successors_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_FATAL("successors should be an array");
      return false;
    }
    for (size_t i_successors = 0; i_successors < successors_xml.size(); i_successors++) {
      XmlRpc::XmlRpcValue successor_xml;
      successor_xml = successors_xml[i_successors];
      if (successor_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_FATAL("element of successors should be an dictionary");
        return false;
      }
      double x = 0;
      double y = 0;
      double theta = 0;
      if (successor_xml.hasMember("x")) {
        x = jsk_topic_tools::getXMLDoubleValue(successor_xml["x"]);
        x += default_x;
      }
      if (successor_xml.hasMember("y")) {
        y = jsk_topic_tools::getXMLDoubleValue(successor_xml["y"]);
        y += default_y;
      }
      if (successor_xml.hasMember("theta")) {
        theta = jsk_topic_tools::getXMLDoubleValue(successor_xml["theta"]);
        theta += default_theta;
      }
      Eigen::Affine3f successor =
        Eigen::Translation3f(inv_lleg_footstep_offset_[0],
                             inv_lleg_footstep_offset_[1],
                             inv_lleg_footstep_offset_[2]) *
        affineFromXYYaw(x, y, theta) *
        Eigen::Translation3f(-inv_rleg_footstep_offset_[0],
                             -inv_rleg_footstep_offset_[1],
                             -inv_rleg_footstep_offset_[2]);
      successors_.push_back(successor);
    }
    ROS_INFO("%lu successors are defined", successors_.size());
    if ((default_x != 0.0) || (default_y != 0.0) || (default_theta != 0.0)) {
      ROS_INFO("default_offset: #f(%f %f %f)", default_x, default_y, default_theta);
    }
    if ((inv_lleg_footstep_offset_[0] != 0) ||
        (inv_lleg_footstep_offset_[1] != 0) ||
        (inv_lleg_footstep_offset_[2] != 0) ) {
      ROS_INFO("left_leg_offset: #f(%f %f %f)",
               - inv_lleg_footstep_offset_[0],
               - inv_lleg_footstep_offset_[1],
               - inv_lleg_footstep_offset_[2]);
    }
    if ((inv_rleg_footstep_offset_[0] != 0) ||
        (inv_rleg_footstep_offset_[1] != 0) ||
        (inv_rleg_footstep_offset_[2] != 0) ) {
      ROS_INFO("right_leg_offset: #f(%f %f %f)",
               - inv_rleg_footstep_offset_[0],
               - inv_rleg_footstep_offset_[1],
               - inv_rleg_footstep_offset_[2]);
    }
    for (size_t i = 0; i < successors_.size(); i++) {
      Eigen::Vector3f tr = successors_[i].translation();
      float roll, pitch, yaw;
      pcl::getEulerAngles(successors_[i], roll, pitch, yaw);
      ROS_INFO("successor_%2.2d: (make-coords :pos (scale 1000 #f(%f %f 0)) :rpy (list %f 0 0))", i, tr[0], tr[1], yaw);
    }
    return true;
  }

  void GridPathPlanner::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool need_to_rebuild_graph = false;
    if (use_pointcloud_model_ != config.use_pointcloud_model) {
      use_pointcloud_model_ = config.use_pointcloud_model;
      need_to_rebuild_graph = true;
    }
    if (use_lazy_perception_ != config.use_lazy_perception) {
      use_lazy_perception_ = config.use_lazy_perception;
      need_to_rebuild_graph = true;
    }
    if (use_local_movement_ != config.use_local_movement) {
      use_local_movement_ = config.use_local_movement;
      need_to_rebuild_graph = true;
    }
    if (resolution_x_ != config.resolution_x) {
      resolution_x_ = config.resolution_x;
      need_to_rebuild_graph = true;
    }
    if (resolution_y_ != config.resolution_y) {
      resolution_y_ = config.resolution_y;
      need_to_rebuild_graph = true;
    }
    if (resolution_theta_ != config.resolution_theta) {
      resolution_theta_ = config.resolution_theta;
      need_to_rebuild_graph = true;
    }
    planning_timeout_ = config.planning_timeout;
    rich_profiling_ = config.rich_profiling;
    parameters_.use_transition_limit = config.use_transition_limit;
    parameters_.use_global_transition_limit = config.use_global_transition_limit;
    parameters_.local_move_x = config.local_move_x;
    parameters_.local_move_y = config.local_move_y;
    parameters_.local_move_theta = config.local_move_theta;
    parameters_.local_move_x_num = config.local_move_x_num;
    parameters_.local_move_y_num = config.local_move_y_num;
    parameters_.local_move_theta_num = config.local_move_theta_num;
    parameters_.local_move_x_offset = config.local_move_x_offset;
    parameters_.local_move_y_offset = config.local_move_y_offset;
    parameters_.local_move_theta_offset = config.local_move_theta_offset;
    parameters_.transition_limit_x = config.transition_limit_x;
    parameters_.transition_limit_y = config.transition_limit_y;
    parameters_.transition_limit_z = config.transition_limit_z;
    parameters_.transition_limit_roll = config.transition_limit_roll;
    parameters_.transition_limit_pitch = config.transition_limit_pitch;
    parameters_.transition_limit_yaw = config.transition_limit_yaw;
    parameters_.global_transition_limit_roll = config.global_transition_limit_roll;
    parameters_.global_transition_limit_pitch = config.global_transition_limit_pitch;
    parameters_.goal_pos_thr = config.goal_pos_thr;
    parameters_.goal_rot_thr = config.goal_rot_thr;
    parameters_.plane_estimation_use_normal              = config.plane_estimation_use_normal;
    parameters_.plane_estimation_normal_distance_weight  = config.plane_estimation_normal_distance_weight;
    parameters_.plane_estimation_normal_opening_angle    = config.plane_estimation_normal_opening_angle;
    parameters_.plane_estimation_min_ratio_of_inliers    = config.plane_estimation_min_ratio_of_inliers;
    parameters_.plane_estimation_max_iterations = config.plane_estimation_max_iterations;
    parameters_.plane_estimation_min_inliers = config.plane_estimation_min_inliers;
    parameters_.plane_estimation_outlier_threshold = config.plane_estimation_outlier_threshold;
    parameters_.support_check_x_sampling = config.support_check_x_sampling;
    parameters_.support_check_y_sampling = config.support_check_y_sampling;
    parameters_.support_check_vertex_neighbor_threshold = config.support_check_vertex_neighbor_threshold;
    parameters_.support_padding_x = config.support_padding_x;
    parameters_.support_padding_y = config.support_padding_y;
    parameters_.skip_cropping = config.skip_cropping;
    footstep_size_x_ = config.footstep_size_x;
    footstep_size_y_ = config.footstep_size_y;
    project_start_state_ = config.project_start_state;
    project_goal_state_ = config.project_goal_state;
    close_list_x_num_ = config.close_list_x_num;
    close_list_y_num_ = config.close_list_y_num;
    close_list_theta_num_ = config.close_list_theta_num;
    profile_period_ = config.profile_period;
    heuristic_ = config.heuristic;
    heuristic_first_rotation_weight_ = config.heuristic_first_rotation_weight;
    heuristic_second_rotation_weight_ = config.heuristic_second_rotation_weight;
    cost_weight_ = config.cost_weight;
    heuristic_weight_ = config.heuristic_weight;
    if (use_obstacle_model_ != config.use_obstacle_model) {
      use_obstacle_model_ = config.use_obstacle_model;
      need_to_rebuild_graph = true;
    }
    parameters_.obstacle_resolution = config.obstacle_resolution;
    if (need_to_rebuild_graph) {
      if (graph_) {             // In order to skip first initialization
        ROS_INFO("re-building graph");
        buildGraph();
      }
    }
  }
  
  void GridPathPlanner::buildGraph()
  {
    graph_.reset(new FootstepGraph(Eigen::Vector3f(resolution_x_,
                                                   resolution_y_,
                                                   resolution_theta_),
                                   use_pointcloud_model_,
                                   use_lazy_perception_,
                                   use_local_movement_,
                                   use_obstacle_model_));
    if (use_pointcloud_model_ && pointcloud_model_) {
      graph_->setPointCloudModel(pointcloud_model_);
    }
    if (use_obstacle_model_ && obstacle_model_) {
      graph_->setObstacleModel(obstacle_model_);
    }
    //graph_->setObstacleResolution(parameters_.obstacle_resolution);
    graph_->setParameters(parameters_);
    graph_->setBasicSuccessors(successors_);
  }

  void GridPathPlanner::setHeuristicPathLine(jsk_recognition_utils::PolyLine &path_line)
  {
    graph_->setHeuristicPathLine(path_line); // copy ???
  }
}
