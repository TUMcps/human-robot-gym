// -*- lsst-c++ -*/
/**
 * @file control_command_translator.h
 * @brief Defines the control command translator class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#ifndef CONTROL_COMMAND_TRANSLATOR_H
#define CONTROL_COMMAND_TRANSLATOR_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>

#include "modrob_workstation/RobotConfigCommanded.h"

namespace safety_shield{

/**
 * @brief Takes the robot motion signal and translates it to a ROS control signal that fits the robot.
 * 
 * You can add more custom robot signals here if your robot takes a different input.
 */
class ControlCommandTranslator{
 private:
  /**
   * @brief Publishes the translated motion command on a ROS topic.
   */
  ros::Publisher pos_command_pub_;

  /**
   * @brief Number of joints that are translated.
   */
  int n_joints_robot_;

 public:
  /**
   * @brief Construct a new Control Command Translator object
   */
  ControlCommandTranslator() {}
  
  /**
   * @brief Construct a new Control Command Translator object
   * 
   * @param pos_command_pub Publishes the translated motion command on a ROS topic.
   * @param n_joints_robot Number of joints that are translated.
   */
  ControlCommandTranslator(const ros::Publisher &pos_command_pub, int n_joints_robot):
    pos_command_pub_(pos_command_pub),
    n_joints_robot_(n_joints_robot) {}

  /**
   * @brief Destroy the Control Command Translator object
   */
  ~ControlCommandTranslator() {}
  
  /**
   * @brief Translate the robot command.
   * 
   * Get a robot config command message and translate it into a command for the arm position controller. 
   * Publish the command for the arm position controller.
   * 
   * @param robot_config_command The joint command in robot config commanded format
   */
  void convertMsg(const modrob_workstation::RobotConfigCommanded* robot_config_command);
};
} // namespace safety_shield
#endif //CONTROL_COMMAND_TRANSLATOR_H