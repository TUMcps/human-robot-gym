#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

#include <visualization_msgs/MarkerArray.h>
#include "custom_robot_msgs/Visualization.h"
#include "modrob_workstation/RobotConfigCommanded.h"

#include "safety_shield/motion.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/rviz_marker.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify_iso.h"
#include "safety_shield/advanced_verify_iso.h"
#include "safety_shield/simulation_time_control.h"
#include "safety_shield/control_command_translator.h"


/**
 * @brief Setup the safety shield with the parameters of the ROS param server.
 * 
 * @param node_handle 
 * @param sample_time 
 * @param nb_joints number of joints
 * @param robot_reach Robot reachable set calculation object
 * @param human_reach Human reachable set calculation object
 * @param verify Verification object
 * @param translator Motion command translation object
 * 
 * @return safety_shield::SafetyShield* 
 */
safety_shield::SafetyShield* setupSafetyShield(ros::NodeHandle& node_handle, 
                                               double sample_time, 
                                               int nb_joints, 
                                               safety_shield::RobotReach* robot_reach,
                                               safety_shield::HumanReach* human_reach,
                                               safety_shield::Verify* verify,
                                               safety_shield::ControlCommandTranslator* translator,
                                               safety_shield::RvizMarker* rviz = nullptr) {
  /// get the parameters
  double t_buff = 0, max_s_stop = 0;
  std::vector<double> v_max_allowed, a_max_allowed, j_max_allowed, a_max_path, j_max_path;
  if (ros::param::has("/buffer_time")) {
      ros::param::get("/buffer_time", t_buff);
  }
  if (ros::param::has("/max_s_stop")) {
      ros::param::get("/max_s_stop", max_s_stop);
  }
  if (ros::param::has("/v_max_allowed")) {
      ros::param::get("/v_max_allowed", v_max_allowed);
  }
  if (ros::param::has("/a_max_allowed")) {
      ros::param::get("/a_max_allowed", a_max_allowed);
  }
  if (ros::param::has("/j_max_allowed")) {
      ros::param::get("/j_max_allowed", j_max_allowed);
  }
  if (ros::param::has("/a_max_traj")) {
      ros::param::get("/a_max_traj", a_max_path);
  }
  if (ros::param::has("/j_max_traj")) {
      ros::param::get("/j_max_traj", j_max_path);
  }
  std::vector<std::vector<double>> q_vals(nb_joints);
  for (int i=1; i <= nb_joints; i++){
      std::string qi = "/q" + std::to_string(i);
      ros::param::get(qi, q_vals[i-1]);
  }
  bool activate_shield = true;
  if (ros::param::has("/modrob/safe")) {
      ros::param::get("/modrob/safe", activate_shield);
  }

  // store the long term trajectory
  std::vector<safety_shield::Motion> long_term_traj;
  for(int i = 0; i < q_vals[0].size(); i++){
      std::vector<double> angles(nb_joints);
      for(int j=0; j < nb_joints; j++){
          angles[j] = q_vals[j][i];
      }
      long_term_traj.push_back(safety_shield::Motion(i*sample_time, angles));
  }
  ROS_DEBUG("Creating LTT...");
  safety_shield::LongTermTraj long_term_trajectory = safety_shield::LongTermTraj(long_term_traj);

  ros::Publisher motion_pub = node_handle.advertise<modrob_workstation::RobotConfigCommanded>("/ns/robot_config_commanded", 1000);

  return new safety_shield::SafetyShield(activate_shield,
    nb_joints, 
    sample_time, 
    t_buff, 
    max_s_stop, 
    v_max_allowed, 
    a_max_allowed, 
    j_max_allowed, 
    a_max_path, 
    j_max_path, 
    long_term_trajectory, 
    motion_pub,
    robot_reach,
    human_reach,
    verify,
    translator,
    rviz);
}

/**
 * @brief Setup the simulation time control
 * 
 * @param node_handle Node handle
 * @param srv gazebo_msgs::GetPhysicsProperties service
 * @param shield The safety shield pointer
 * @param sample_time Sample time of the safety shield
 * @param n_sim_steps_per_sample_time number of simulation steps per safety shield step
 * @return safety_shield::SimulationTimeControl* 
 */
safety_shield::SimulationTimeControl* setupSimulationTimeControl(ros::NodeHandle& node_handle, gazebo_msgs::GetPhysicsProperties& srv,
    safety_shield::SafetyShield* shield, double sample_time, double n_sim_steps_per_sample_time) {
  bool step_control = true;
  if (ros::param::has("/step_control")) {
    ros::param::get("/step_control", step_control);
  }
  double initial_max_update_rate=10000;
  int n_delays_queue=5;
  double speed_factor=0.1;
  double max_max_update_rate=15000;
  double max_delay_time=0.5;
  double first_reduction_after=10;
  double time_between_reductions=1;
  double time_between_speedups=10;
  if (!step_control) {
    if (ros::param::has("/initial_max_update_rate")) {
      ros::param::get("/initial_max_update_rate", initial_max_update_rate);
    }
    if (ros::param::has("/n_delays_queue")) {
      ros::param::get("/n_delays_queue", n_delays_queue);
    }
    if (ros::param::has("/speed_factor")) {
      ros::param::get("/speed_factor", speed_factor);
    }
    if (ros::param::has("/max_max_update_rate")) {
      ros::param::get("/max_max_update_rate", max_max_update_rate);
    }
    if (ros::param::has("/max_delay_time")) {
      ros::param::get("/max_delay_time", max_delay_time);
    }
    if (ros::param::has("/first_reduction_after")) {
      ros::param::get("/first_reduction_after", first_reduction_after);
    }
    if (ros::param::has("/time_between_reductions")) {
      ros::param::get("/time_between_reductions", time_between_reductions);
    }
    if (ros::param::has("/time_between_speedups")) {
      ros::param::get("/time_between_speedups", time_between_speedups);
    }
  }
  if (step_control) {
    return new safety_shield::SimulationTimeControl(shield, sample_time, n_sim_steps_per_sample_time);
  } else { 
    // Set initial max update rate
    ros::ServiceClient set_physics_properties_client = node_handle.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
    gazebo_msgs::SetPhysicsProperties set_physics_srv;
    set_physics_srv.request.gravity = srv.response.gravity;
    set_physics_srv.request.max_update_rate = initial_max_update_rate;
    set_physics_srv.request.ode_config = srv.response.ode_config;
    set_physics_srv.request.time_step = srv.response.time_step;
    // Set the initial update rate
    set_physics_properties_client.call(set_physics_srv);
    // Simulation speed control
    return new safety_shield::SimulationTimeControl(shield, 
        set_physics_properties_client,
        set_physics_srv,
        sample_time, 
        initial_max_update_rate,
        n_delays_queue,
        speed_factor,
        max_max_update_rate,
        max_delay_time,
        first_reduction_after,
        time_between_reductions,
        time_between_speedups);
  }
}

/**
 * @brief Setup the robot reachability analysis
 * 
 * @param nb_joints number of joints
 * @param robot_name robot name
 * 
 * @return safety_shield::RobotReach* 
 */
safety_shield::RobotReach* setupRobotReach(int nb_joints, std::string robot_name) {
  /// get the robot's parameters
  double init_roll=0, init_pitch=0, init_yaw=0, init_x=0, init_y=0, init_z=0;
  std::vector<double> transformation_matrices, enclosures;
  
  if (ros::param::has("/" + robot_name + "/transformation_matrices")) {
      ros::param::get("/" + robot_name + "/transformation_matrices", transformation_matrices);
  }
  if (ros::param::has("/" + robot_name + "/enclosures")) {
      ros::param::get("/" + robot_name + "/enclosures", enclosures);
  }
  if (ros::param::has("/" + robot_name + "/init_roll")) {
    ros::param::get("/" + robot_name + "/init_roll", init_roll);
  }
  if (ros::param::has("/" + robot_name + "/init_pitch")) {
    ros::param::get("/" + robot_name + "/init_pitch", init_pitch);
  }
  if (ros::param::has("/" + robot_name + "/init_yaw")) {
    ros::param::get("/" + robot_name + "/init_yaw", init_yaw);
  }
  if (ros::param::has("/" + robot_name + "/init_x")) {
    ros::param::get("/" + robot_name + "/init_x", init_x);
  }
  if (ros::param::has("/" + robot_name + "/init_y")) {
    ros::param::get("/" + robot_name + "/init_y", init_y);
  }
  
  if (ros::param::has("/" + robot_name + "/init_z")) {
    ros::param::get("/" + robot_name + "/init_z", init_z);
  }
  
  ROS_INFO("Creating robot object...");
  /// initialize the robot
  return new safety_shield::RobotReach(transformation_matrices, nb_joints, enclosures,
                init_x, init_y, init_z, init_roll, init_pitch, init_yaw);

}


/**
 * @brief Create a human bodies object
 * 
 * @param[in] bodies List of bodies to create. Contains body name, proximal-, distal joint, and thickness.
 * @param[in] joint_ids Maps joint measruement ids to joint names
 * @param[out] body_link_joints Maps joint pairs to body names
 * @param[out] thickness Maps thickness to body names
 */
void createBodies(XmlRpc::XmlRpcValue& bodies, const std::map<std::string, int>& joint_ids, 
                  std::map<std::string, safety_shield::jointPair>& body_link_joints, std::map<std::string, double>& thickness) {
  ROS_ASSERT(bodies.getType()==XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i<bodies.size(); i++) {
    auto& body = bodies[i];
    ROS_ASSERT(body.getType()==XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::iterator info=body.begin(); info!=body.end(); ++info) {
      std::string body_name = info->first;
      int proximal_joint_id = -1;
      int distal_joint_id = -1;
      double r = 0.0;
      ROS_ASSERT(info->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
      for (int j = 0; j<info->second.size(); j++) {
        auto& feature = info->second[j];
        ROS_ASSERT(feature.getType()==XmlRpc::XmlRpcValue::TypeStruct);
        for (XmlRpc::XmlRpcValue::iterator f=feature.begin(); f!=feature.end(); ++f) {
          std::string feature_name = f->first;
          if (feature_name == "proximal") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string proximal = f->second;
            ROS_ASSERT(joint_ids.count(proximal) > 0);
            proximal_joint_id = joint_ids.at(proximal);
          } else if (feature_name == "distal") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string distal = f->second;
            ROS_ASSERT(joint_ids.count(distal) > 0);
            distal_joint_id = joint_ids.at(distal);
          } else if (feature_name == "thickness") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeDouble);
            r = f->second;
          }
        }
      }
      // Create map entries
      ROS_INFO_STREAM("Creating body " << body_name << " with joint pair (" << proximal_joint_id << ", " << distal_joint_id << ") and thickness " << r);
      body_link_joints[body_name] = safety_shield::jointPair(proximal_joint_id, distal_joint_id);
      thickness[body_name] = r;
    }
  }
}

/**
 * @brief Create a human extremities object
 * 
 * @param[in] extemities List of extremities to create. Contains joint name and id.
 * @param[in] joint_ids Maps joint measruement ids to joint names
 * @param[out] shoulder_ids List of shoulder joints
 * @param[out] elbow_ids List of elbow joints
 * @param[out] wrist_names List of wrist joints
 */
void createExtremities(XmlRpc::XmlRpcValue& extemities, const std::map<std::string, int>& joint_ids,
                      std::vector<double>& shoulder_ids, std::vector<double>& elbow_ids, std::vector<std::string>& wrist_names) {
  ROS_ASSERT(extemities.getType()==XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i<extemities.size(); i++) {
    auto& extr = extemities[i];
    ROS_ASSERT(extr.getType()==XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::iterator info=extr.begin(); info!=extr.end(); ++info) {
      std::string extr_name = info->first;
      int shoulder_joint_id = -1;
      int elbow_joint_id = -1;
      std::string wrist_body_name = "";
      ROS_ASSERT(info->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
      for (int j = 0; j<info->second.size(); j++) {
        auto& feature = info->second[j];
        ROS_ASSERT(feature.getType()==XmlRpc::XmlRpcValue::TypeStruct);
        for (XmlRpc::XmlRpcValue::iterator f=feature.begin(); f!=feature.end(); ++f) {
          std::string feature_name = f->first;
          if (feature_name == "shoulder") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string shoulder = f->second;
            ROS_ASSERT(joint_ids.count(shoulder) > 0);
            shoulder_joint_id = joint_ids.at(shoulder);
          } else if (feature_name == "elbow") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string elbow = f->second;
            ROS_ASSERT(joint_ids.count(elbow) > 0);
            elbow_joint_id = joint_ids.at(elbow);
          } else if (feature_name == "wristBody") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string w = f->second;
            wrist_body_name = w;
          }
        }
      }
      // Create map entries
      ROS_INFO_STREAM("Creating extremity " << extr_name << " with shoulder joint " << shoulder_joint_id << ", elbow joint " << elbow_joint_id << " and body name " << wrist_body_name);
      shoulder_ids.push_back(shoulder_joint_id);
      elbow_ids.push_back(elbow_joint_id);
      wrist_names.push_back(wrist_body_name);
    }
  }
}

/**
 * @brief Setup the human reach object
 * 
 * @return safety_shield::HumanReach* 
 */
safety_shield::HumanReach* setupHumanReach() {
    // get sensor information
  double measurement_error_pos = 0, measurement_error_vel = 0, delay = 0;
  if (ros::param::has("/motion_capture/measurement_error_pos")) {
    ros::param::get("/motion_capture/measurement_error_pos", measurement_error_pos);
  }
  if (ros::param::has("/motion_capture/measurement_error_vel")) {
    ros::param::get("/motion_capture/measurement_error_vel", measurement_error_vel);
  }
  if (ros::param::has("/motion_capture/delay")) {
    ros::param::get("/motion_capture/delay", delay);
  }

  std::vector<std::string> joint_names;
  if (ros::param::has("/motion_capture/joint_names")) {
    ros::param::get("/motion_capture/joint_names", joint_names);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_names not in parameter server!");
  }
  std::map<std::string, int> joint_ids;
  for (int i = 0; i < joint_names.size(); i++) {
      joint_ids[joint_names[i]] = i;
  }

  std::vector<double> joint_v_max;
  if (ros::param::has("/motion_capture/joint_v_max")) {
    ros::param::get("/motion_capture/joint_v_max", joint_v_max);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_v_max not in parameter server!");
  }

  std::vector<double> joint_a_max;
  if (ros::param::has("/motion_capture/joint_a_max")) {
    ros::param::get("/motion_capture/joint_a_max", joint_a_max);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_a_max not in parameter server!");
  }


  /// Create bodies
  std::map<std::string, safety_shield::jointPair> body_link_joints;
  std::map<std::string, double> thickness;
  XmlRpc::XmlRpcValue bodies;
  if (!ros::param::get("/motion_capture/bodies", bodies)) {
    ROS_ERROR("Parameter /motion_capture/bodies not in parameter server!");
  }
  createBodies(bodies, joint_ids, body_link_joints, thickness);
  
  /// Create extremities
  std::vector<double> shoulder_ids;
  std::vector<double> elbow_ids;
  std::vector<std::string> wrist_names;
  XmlRpc::XmlRpcValue extremities;
  if (!ros::param::get("/motion_capture/extremities", extremities)) {
    ROS_ERROR("Parameter /motion_capture/extremities not in parameter server!");
  }
  createExtremities(extremities, joint_ids, shoulder_ids, elbow_ids, wrist_names);

  return new safety_shield::HumanReach(joint_names.size(), body_link_joints, 
                            thickness, joint_v_max, joint_a_max, 
                            shoulder_ids, elbow_ids, wrist_names,
                            measurement_error_pos, measurement_error_vel, delay);
}


/**
 * @brief Setup verification object
 * 
 * @return safety_shield::Verify*
 */
safety_shield::Verify* setupVerifyISO() {
  bool advanced_verify = false;
  if (ros::param::has("advanced_verify_iso")) {
    ros::param::get("advanced_verify_iso", advanced_verify);
  }
  ROS_WARN("Using advanced verify ISO = %i", advanced_verify);
  // Create verify ISO object and message synchronizer
  if (advanced_verify) {
    safety_shield::AdvancedVerifyISO* p = new safety_shield::AdvancedVerifyISO();
    return p;
  } else {
    return new safety_shield::VerifyISO();
  }
}


/**
 * @brief Setup the control command translator object
 * 
 * This sends the motion commands to the robot via ROS
 * This is not yet written dynamically, so only one type of ROS msg is possible. 
 * TODO: Rewrite to allow flexible ROS topics
 * 
 * @param nb_joints number of joints
 * @param robot_name robot name
 * @return safety_shield::ControlCommandTranslator* 
 */
safety_shield::ControlCommandTranslator* setupControlCommandTranslator(ros::NodeHandle& node_hanlde, int nb_joints, std::string robot_name) {
  std::string publish_topic = "/" + robot_name + "/arm_position_controller/command";
  ros::Publisher pos_command_pub = node_hanlde.advertise<std_msgs::Float64MultiArray>(publish_topic, 1000);
  return new safety_shield::ControlCommandTranslator(pos_command_pub, nb_joints);
}


/**
 * @brief Setup the Rviz Visualization Publisher Class
 * 
 * @param node_handle 
 * @return safety_shield::RvizMarker* 
 */
safety_shield::RvizMarker* setupRvizMarker(ros::NodeHandle& node_handle) {
  // Define publishers
  ros::Publisher robot_visu_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/robot_marker_array", 1000);
  ros::Publisher human_cylinder_visu_pub = node_handle.advertise<visualization_msgs::MarkerArray>(
      "/human_cylinder_marker_array", 1000);
  ros::Publisher human_reach_visu_pub_P = node_handle.advertise<visualization_msgs::MarkerArray>(
      "/human_reach_marker_array/P", 1000);
  ros::Publisher human_reach_visu_pub_V = node_handle.advertise<visualization_msgs::MarkerArray>(
      "/human_reach_marker_array/V", 1000);
  ros::Publisher human_reach_visu_pub_A = node_handle.advertise<visualization_msgs::MarkerArray>(
      "/human_reach_marker_array/A", 1000);

  // Create rviz marker object
  return new safety_shield::RvizMarker(robot_visu_pub, 
      human_cylinder_visu_pub, 
      human_reach_visu_pub_P, 
      human_reach_visu_pub_V, 
      human_reach_visu_pub_A);
}

/**
 * @brief Main function of the safety shield node
 * 
 * Creates the following objects
 *    - Safety shield
 *    - Simulation Time Control
 *    - Robot Reach
 *    - Human Reach
 *    - Verification
 *    - Robot Output
 * 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "safety_shield");
  ros::NodeHandle node_handle;

  double sample_time = 0.004;

  if (ros::param::has("/sample_time")) {
    ros::param::get("/sample_time", sample_time);
    ROS_ASSERT(sample_time > 0);
  }

  // Get the max step size (dt sim time per update cycle) from gazbeo
  ros::ServiceClient get_physics_properties_client = node_handle.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
  gazebo_msgs::GetPhysicsProperties srv;
  double max_time_step = 0;
  double max_update_rate = 0;
  while (max_time_step <= 0) {
    get_physics_properties_client.call(srv);
    max_time_step = srv.response.time_step;
    max_update_rate = srv.response.max_update_rate;
    ros::Duration(0.1).sleep();
    ROS_WARN_STREAM("Gazebo physics parameter time step = " << srv.response.time_step << " not available yet. Waiting for GetPhysicsProperties service...");
  }
  // Set the sample time to be a multiplicative of the max_time_step 
  int n_sim_steps_per_sample_time = 1;
  if (sample_time >= max_time_step) {
    double d = sample_time / max_time_step;
    n_sim_steps_per_sample_time = floor(d);
    sample_time = max_time_step * n_sim_steps_per_sample_time;
  } else {
    sample_time = max_time_step;
  }
  ROS_INFO_STREAM("n_sim_steps_per_sample_time = " << n_sim_steps_per_sample_time << " sample time = " << sample_time);

  std::string robot_name = "modrob0";
  if (ros::param::has("/robot_name")) {
    ros::param::get("/robot_name", robot_name);
  }
  int nb_joints = 0;
  if (ros::param::has("/" + robot_name + "/nb_joints")) {
    ros::param::get("/" + robot_name + "/nb_joints", nb_joints);
  }

  bool visualize_rviz = false;
  if (ros::param::has("/visualize_rviz")) {
    ros::param::get("/visualize_rviz", visualize_rviz);
  }
  safety_shield::RvizMarker* rviz;
  if (visualize_rviz) {
    rviz = setupRvizMarker(node_handle);
  } else {
    rviz = nullptr;
  }
  
  safety_shield::RobotReach* robot_reach = setupRobotReach(nb_joints, robot_name);

  safety_shield::HumanReach* human_reach = setupHumanReach();
  ros::Subscriber sub_human_joint_pos_ = node_handle.subscribe("/human_joint_pos", 1000, &safety_shield::HumanReach::measurement, human_reach);

  safety_shield::Verify* verify = setupVerifyISO();

  safety_shield::ControlCommandTranslator* translator = setupControlCommandTranslator(node_handle, nb_joints, robot_name);

  safety_shield::SafetyShield* shield = setupSafetyShield(node_handle, sample_time, nb_joints, robot_reach, human_reach, verify, translator, rviz);
  ros::Subscriber sub_new_goal = node_handle.subscribe("/" + robot_name + "/new_goal_motion", 1000, &safety_shield::SafetyShield::newLongTermTrajectory, shield);

  safety_shield::SimulationTimeControl* sim_control = setupSimulationTimeControl(node_handle, srv, shield, sample_time, n_sim_steps_per_sample_time);
  ros::Subscriber sub = node_handle.subscribe("/initialisation", 1000, &safety_shield::SimulationTimeControl::init, sim_control);

  // Activate node message receiving and sending
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();
  //ros::spin();

  return true;
}
