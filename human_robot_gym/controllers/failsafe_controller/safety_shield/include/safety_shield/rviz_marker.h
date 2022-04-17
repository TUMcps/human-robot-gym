// -*- lsst-c++ -*/
/**
 * @file rviz_marker.h
 * @brief Defines the rviz visualization class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <exception>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "global_library/global_library.h"
#include "custom_robot_msgs/Visualization.h"
#include "custom_robot_msgs/CapsuleArray.h"
#include "custom_robot_msgs/StartGoalCapsuleArray.h"


#ifndef RVIZ_MARKER_H
#define RVIZ_MARKER_H

namespace safety_shield {

/**
 * @brief Converts incoming human and robot capsules to rviz visualization messages
 */
class RvizMarker{
  enum {
    ROBOT = 0u,
    HUMAN_CYLINDER = 1u,
    HUMAN_REACH = 2u
  };
  
 private:
  /**
   * @brief Robot visualization markers
   */
  visualization_msgs::MarkerArray robot_markers_; 

  /**
   * @brief Human polycapsule markers
   */
  visualization_msgs::MarkerArray human_cylinder_markers_; 

  /**
   * @brief Human capsule markers for position
   */
  visualization_msgs::MarkerArray human_reach_markers_P_; 

  /**
   * @brief Human capsule markers for velocity
   */
  visualization_msgs::MarkerArray human_reach_markers_V_; 

  /**
   * @brief Human capsule markers for acceleration
   */
  visualization_msgs::MarkerArray human_reach_markers_A_; 

  /**
   * @brief publishes the visualization of the reachable set of the robot
   */
  ros::Publisher robot_visu_pub_;

  /**
   * @brief publishes the visualization of the reachable set of the human polycapsules
   */
  ros::Publisher human_cylinder_visu_pub_;

  /**
   * @brief publishes the visualization of the reachable set of the human capsules (position)
   */
  ros::Publisher human_reach_visu_pub_P_;

  /**
   * @brief publishes the visualization of the reachable set of the human capsules (velocity)
   */
  ros::Publisher human_reach_visu_pub_V_;

  /**
   * @brief publishes the visualization of the reachable set of the human capsules (acceleration)
   */
  ros::Publisher human_reach_visu_pub_A_;
  
  /**
   * @brief Updates the markers associated to the robot's modules
   * 
   * @param capsules the robot's capsules
   */
  void robotUpdate(custom_robot_msgs::CapsuleArray &capsules);

  /**
   * @brief Adds a given number of marker in the markers list
   * @param[in] markers          The marker array to add points to 
   * @param[in] nb_points_to_add the number of markers to add
   * @param[in] shape_type       the type of the marker 
   * @param[in] color_type       ROBOT, HUMAN_CYLINDER, or HUMAN_REACH  
   */
  void createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, int color_type);

  /**
   * @brief Update the given marker array with the given capsules
   * @param[in] markers          The marker array to update the points in
   * @param[in] capsules         The new capsules
   */
  void createCapsules(visualization_msgs::MarkerArray& markers, 
      const custom_robot_msgs::CapsuleArray* capsules);

  /**
   * @brief Update a marker with a sphere
   * @param[in] pos              The center of the sphere
   * @param[in] radius           The radius
   * @param[in] stamp            Time for header stamp
   * @param[in] marker           The marker to update
   */
  void createSphere(const geometry_msgs::Point& pos, double radius, 
      const ros::Time& stamp, visualization_msgs::Marker& marker);

  /**
   * @brief Update a marker with a cylinder
   * @param[in] seg              A segment with the two end points of the center line of the cylinder.
   * @param[in] radius           The radius
   * @param[in] stamp            Time for header stamp
   * @param[in] marker           The marker to update
   */
  void createCylinder(const custom_robot_msgs::Segment& seg, double radius, 
      const ros::Time& stamp, visualization_msgs::Marker& marker);

  /**
   * @brief Calculates the human reach capsule markers
   */
  void human_reach_calc(const custom_robot_msgs::CapsuleArray* data, 
      visualization_msgs::MarkerArray& human_reach_markers);

 public:
  /**
   * An empty RvizMarker constructor
   */
  RvizMarker() {}
  
  /**
   * @brief RvizMarker constructor
   * @param[in] robot_visu_pub publishes the visualization of the reachable set of the robot
   * @param[in] human_cylinder_visu_pub publishes the visualization of the reachable set of the human polycapsules
   * @param[in] human_reach_visu_pub_X publishes the visualization of the reachable set of the human capsules
   */
  RvizMarker(ros::Publisher robot_visu_pub, 
      ros::Publisher human_cylinder_visu_pub, 
      ros::Publisher human_reach_visu_pub_P, 
      ros::Publisher human_reach_visu_pub_V, 
      ros::Publisher human_reach_visu_pub_A);

  /**
   * @brief A RvizMarker destructor
   */
  ~RvizMarker() {}

  /**
   * @brief Reads the robot capsule data and updates and sends the rviz markers
   */
  void robot_callback(const custom_robot_msgs::CapsuleArray* data);

  /**
   * @brief Reads the robot capsule data and updates and sends the rviz markers
   */
  void advanced_robot_callback(const custom_robot_msgs::StartGoalCapsuleArray* data);

  /**
   * @brief Reads the human cylinder polycapsule data and updates and sends the rviz markers
   */
  void human_cylinder_callback(const custom_robot_msgs::PolycapsuleArray* data);
  
  /**
   * @brief Reads the human reach capsule data and updates and sends the rviz markers
   */
  void human_reach_callback_p(const custom_robot_msgs::CapsuleArray* data);

  /**
   * @brief Reads the human reach capsule data and updates and sends the rviz markers
   */
  void human_reach_callback_v(const custom_robot_msgs::CapsuleArray* data);
  
  /**
   * @brief Reads the human reach capsule data and updates and sends the rviz markers
   */
  void human_reach_callback_a(const custom_robot_msgs::CapsuleArray* data);
  
};
} // namespace safety_shield

#endif // RVIZ_MARKER_H
