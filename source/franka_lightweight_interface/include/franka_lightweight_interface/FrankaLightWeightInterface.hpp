#pragma once

#include <chrono>
#include <iostream>
#include <mutex>

#include <franka/model.h>
#include <franka/robot.h>

#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>
#include <state_representation/space/joint/JointState.hpp>

namespace frankalwi {

struct CollisionBehaviour {
  std::array<double, 7> ltta;///lower_torque_thresholds_acceleration
  std::array<double, 7> utta;///upper_torque_thresholds_acceleration
  std::array<double, 7> lttn;///lower_torque_thresholds_nominal
  std::array<double, 7> uttn;///upper_torque_thresholds_nominal
  std::array<double, 6> lfta;///lower_force_thresholds_acceleration
  std::array<double, 6> ufta;///upper_force_thresholds_acceleration
  std::array<double, 6> lftn;///lower_force_thresholds_nominal
  std::array<double, 6> uftn;///upper_force_thresholds_nominal
};

/**
 * @class FrankaLightweightInterface
 * @brief Class to define an interface with the Franka panda robots
 *
 */
class FrankaLightWeightInterface {
public:
  /**
   * @brief Constructor for the FrankaLightWeightInterface class
   * @param robot_ip ip address of the robot to control
   */
  explicit FrankaLightWeightInterface(
      std::string robot_ip, communication_interfaces::sockets::ZMQCombinedSocketsConfiguration state_command_config,
      std::string prefix);

  /**
   * @brief Set the joint damping gains.
   * @param[in] joint_damping_gains The desired array of damping gains per joint.
   */
  void set_joint_damping(const Eigen::Array<double, 7, 1>& joint_damping_gains);

  /**
   * @brief Set the joint damping gains.
   * @param[in] joint_damping_gains The desired array of damping gains per joint.
   */
  void set_joint_damping(const std::array<double, 7>& joint_damping_gains);

  /**
   * @brief Set the joint impedance.
   * @param[in] joint_impedance_values The desired array of impedance values per joint.
   */
  void set_joint_impedance(const Eigen::Array<double, 7, 1>& joint_impedance_values);

  /**
   * @brief Set the joint impedance.
   * @param[in] joint_impedance_values The desired array of impedance values per joint.
   */
  void set_joint_impedance(const std::array<double, 7>& joint_impedance_values);

  /**
   * @brief Set the collision behaviour.
   *
   * @details Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * The new values only take effect when a controller is started.
   *
   * @param[in] lower_torque_thresholds_acceleration Contact torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_acceleration Collision torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] lower_torque_thresholds_nominal Contact torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_nominal Collision torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds_acceleration Contact force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] upper_force_thresholds_acceleration Collision force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] lower_force_thresholds_nominal Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds_nominal Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   */
  void set_collision_behaviour(
      const std::array<double, 7>& lower_torque_thresholds_acceleration,
      const std::array<double, 7>& upper_torque_thresholds_acceleration,
      const std::array<double, 7>& lower_torque_thresholds_nominal,
      const std::array<double, 7>& upper_torque_thresholds_nominal,
      const std::array<double, 6>& lower_force_thresholds_acceleration,
      const std::array<double, 6>& upper_force_thresholds_acceleration,
      const std::array<double, 6>& lower_force_thresholds_nominal,
      const std::array<double, 6>& upper_force_thresholds_nominal);

  /**
   * @brief Set the collision behaviour.
   * @copydetails FrankaLightWeightInterface::set_collision_behaviour(<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&)
   * @param collision_behaviour The collision behaviour structure
   * containing lower and upper torque and force thresholds.
   * @see FrankaLightWeightInterface::set_collision_behaviour(<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&)
   */
  void set_collision_behaviour(const CollisionBehaviour& collision_behaviour);

  /**
   * @brief Initialize the connection to the robot
   */
  void init();

  /**
   * @brief Threaded function that run a controller based on the value in the active_controller enumeration
   */
  void run_controller();

private:
  /**
   * @brief Poll the ZMQ socket subscription for a new joint torque command from an external controller
   */
  void poll_external_command();

  /**
  * @brief Read and publish robot state to the ZMQ socket for an external controller or observer to receive
  */
  void read_and_publish_robot_state(const franka::RobotState& robot_state);

  /**
   * @brief Read and publish the robot state while no control commands are received
   */
  void run_state_publisher();

  /**
   * @brief Run the joint velocities controller
   * that reads commands from the joint velocities subscription
   */
  void run_joint_velocities_controller();

  /**
   * @brief Run the joint torques controller
   * that reads commands from the joint torques subscription
   */
  void run_joint_torques_controller();

  void print_state() const;

  std::string prefix_;                         ///< prefix of the robot joints
  std::string robot_ip_;                       ///< ip of the robot to connect to
  std::unique_ptr<franka::Robot> franka_robot_;///< robot object to send command to
  std::unique_ptr<franka::Model> franka_model_;///< model object of the robot
  bool connected_;
  bool shutdown_;
  state_representation::JointState state_;
  std::shared_ptr<state_representation::JointState> command_;
  communication_interfaces::sockets::ZMQPublisherSubscriber sockets_;
  Eigen::ArrayXd joint_damping_gains_;
  std::array<double, 7> joint_impedance_values_;
  CollisionBehaviour collision_behaviour_;
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::milliseconds command_timeout_ = std::chrono::milliseconds(500);
  std::mutex mutex_;
};

inline void FrankaLightWeightInterface::print_state() const {
  std::cout << "Joint state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->state_ << std::endl;
  if (this->command_) {
    std::cout << "--------------------" << std::endl;
    std::cout << "Command:" << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << this->command_ << std::endl;
  }
  std::cout << "####################" << std::endl;
}
}// namespace frankalwi
