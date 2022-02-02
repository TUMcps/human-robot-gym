#include "safety_shield/control_command_translator.h"

namespace safety_shield
{
void ControlCommandTranslator::convertMsg(
    const modrob_workstation::RobotConfigCommanded* robot_config_command) {
  ROS_DEBUG_STREAM("ControlCommandTranslator::convertMsg");
  //robot_config_command: bool tool_activation
  //                      JointConfigCommanded[] joint_moves
  try {
    std_msgs::MultiArrayDimension msg_dim = std_msgs::MultiArrayDimension();
    msg_dim.label = "joint_positions";
    msg_dim.size = n_joints_robot_;
    msg_dim.stride = n_joints_robot_;
    std_msgs::MultiArrayLayout msg_layout = std_msgs::MultiArrayLayout();
    msg_layout.dim.push_back(msg_dim);
    msg_layout.data_offset = 0;
    std_msgs::Float64MultiArrayPtr msg(new std_msgs::Float64MultiArray());
    msg->layout = msg_layout;
    for (int i=0; i<std::size(robot_config_command->joint_moves); i++) {
      if (i<n_joints_robot_) {
        msg->data.push_back(robot_config_command->joint_moves[i].joint_angle);
      } else {
        break;
      }
    }
    pos_command_pub_.publish(msg);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in ControlCommandTranslator::convertMsg: " << exc.what());
  }
}

} // namespace safety_shield