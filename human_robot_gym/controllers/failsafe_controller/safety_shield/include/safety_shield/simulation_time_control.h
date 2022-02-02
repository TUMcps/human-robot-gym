// -*- lsst-c++ -*/
/**
 * @file simulation_time_control.h
 * @brief defined all the structure used in the simulation time control
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */


#include <exception>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_msgs/Empty.h>

#include "safety_shield/safety_shield.h"

#ifndef SIMULATION_TIME_CONTROL_H
#define SIMULATION_TIME_CONTROL_H

namespace safety_shield {
/**
 * @brief Controls the speed of the gazebo simulation
 */
class SimulationTimeControl{
private: 

  /**
   * @brief Pointer to the safety shield object.
   * 
   * The simulation time control calls the step() function of the safety shield.
   */
  SafetyShield* safety_shield_;

  /**
   * @brief Gazebo control node
   */
  gazebo::transport::NodePtr gazebonode_;

  /**
   * @brief Publish the step control information
   */
  gazebo::transport::PublisherPtr world_control_pub_; 

  /**
   * @brief Whether to use continuous control or step control
   * 
   */
  bool use_step_control_;

  /**
   * @brief The time when the loop begins
   */
  ros::Time cycle_begin_time_;

  /**
   * @brief The sample time of the online verification calcualation
   */
  double sample_time_;

  /**
   * @brief The current maximum update rate of the simulation
   */
  double max_update_rate_;

  /**
   * @brief The initial maximum update rate of the simulation
   */
  double initial_max_update_rate_;

  /**
   * @brief A FIFO queue holding the past n times where the OV calculation was slower than the simulation
   */
  std::queue<ros::Time> delay_queue_;

  /**
   * @brief Size of the delay_queue
   */
  int n_delays_queue_;

  /**
   * @brief Speedup/Slow down factor
   */
  double speed_factor_;

  /**
   * @brief Max simulation speed
   */
  double max_max_update_rate_;

  /**
   * @brief Number of simulation steps per OV calculation
   */
  int n_sim_steps_per_sample_time_;

  /**
   * @brief If the delay_queue(0)-delay_queue(5) < max_delay_time, Then reduce the simulation speed.
   */
  ros::Duration max_delay_time_;

  /**
   * @brief Do the first simulation speed reduction earliest after x seconds
   */
  ros::Time first_reduction_after_;

  /**
   * @brief The last time the simulation speed was reduced
   */
  ros::Time last_reduction_;

  /**
   * @brief The minimum time between two reductions (Simulation needs some time to reduce the speed)
   */
  ros::Duration time_between_reductions_;
 
  /**
   * @brief The minimum time between two speedups
   */
  ros::Duration time_between_speedups_;

  /**
   * @brief The set gazebo speed service client
   */
  ros::ServiceClient set_physics_properties_client_;

  /**
   * @brief The set gazebo speed service message
   */
  gazebo_msgs::SetPhysicsProperties set_physics_srv_;
  

public:
  /**
   * @brief Empty constructor
   */
  SimulationTimeControl(){}

  /**
   * @brief Contructor for simulation speed control
   * @param[in] safety_shield Pointer to safety shield object
   * @param[in] set_physics_properties_client The set gazebo speed service client
   * @param[in] set_physics_srv The set gazebo speed service message
   * @param[in] sample_time The sample time of the online verification calcualation
   * @param[in] initial_max_update_rate The initial maximum update rate of the simulation
   * @param[in] n_delays_queue Size of the delay_queue
   * @param[in] speed_factor Speedup/Slow down factor
   * @param[in] max_max_update_rate Max simulation speed
   * @param[in] max_delay_time If the delay_queue(0)-delay_queue(5) < max_delay_time, Then reduce the simulation speed.
   * @param[in] first_reduction_after Do the first simulation speed reduction earliest after x seconds
   * @param[in] time_between_reductions The minimum time between two reductions (Simulation needs some time to reduce the speed)
   * @param[in] time_between_speedups The minimum time between two speedups
   */
  SimulationTimeControl(SafetyShield* safety_shield,
                        const ros::ServiceClient& set_physics_properties_client,
                        gazebo_msgs::SetPhysicsProperties& set_physics_srv,
                        double sample_time=0.004, 
                        double initial_max_update_rate=10000,
                        int n_delays_queue=5,
                        double speed_factor=0.1,
                        double max_max_update_rate=15000,
                        double max_delay_time=0.5,
                        double first_reduction_after=10,
                        double time_between_reductions=1,
                        double time_between_speedups=10);

  /**
   * @brief Constructor for simulation step control
   * @param[in] safety_shield Pointer to safety shield object
   * @param[in] sample_time Sample time simulation
   * @param[in] n_sim_steps_per_sample_time Fixed number of steps to do in each simulation cycle 
   */
  SimulationTimeControl(SafetyShield* safety_shield,
                        double sample_time, 
                        int n_sim_steps_per_sample_time);

  
  /**
   * @brief Destructor
   */
  ~SimulationTimeControl() {}

  /**
   * @brief start the time control
   * 
   */
  void init(const std_msgs::EmptyConstPtr& start);

  /**
   * @brief Control the simulation speed
   *
   * This is faster but less accurate than the step sim control
   * Slow down the simulation speed if the OV calculation is often slower than the simulation.
   * Speed up the simulation speed from time to time to see if the last slow down was unnecessary.
   */
  void simControl();

  /**
   * @brief Control the simulation and safety shield time management.
   *
   * Control the simulation speed with a simulation step approach
   * This is slower but more accurate than simControl.
   */
  void stepSimControl();
};
} // namespace safety_shield
#endif // SIMULATION_TIME_CONTROL_H
