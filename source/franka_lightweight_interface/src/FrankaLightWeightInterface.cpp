#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

#include <franka/exception.h>

#include <clproto.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <stdexcept>
#include <thread>

using namespace state_representation;

namespace frankalwi {

static CollisionBehaviour default_collision_behaviour() {
  return {{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}};
}

static Eigen::Array<double, 7, 1> default_joint_damping_gains() {
  Eigen::ArrayXd gains = Eigen::ArrayXd(7);
  gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;
  return gains;
}

static std::array<double, 7> default_joint_impedance_values() {
  return {2000, 2000, 2000, 1500, 1500, 1000, 1000};
}

FrankaLightWeightInterface::FrankaLightWeightInterface()
    : connected_(false),
      shutdown_(false),
      joint_damping_gains_(default_joint_damping_gains()),
      joint_impedance_values_(default_joint_impedance_values()),
      collision_behaviour_(default_collision_behaviour()) {}

void FrankaLightWeightInterface::init(
    std::string robot_ip, communication_interfaces::sockets::ZMQCombinedSocketsConfiguration state_command_config,
    std::string prefix) {
  // create connection to the robot
  this->franka_robot_ = std::make_unique<franka::Robot>(robot_ip);
  this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());

  this->connected_ = true;

  this->sockets_ = std::make_shared<communication_interfaces::sockets::ZMQPublisherSubscriber>(state_command_config);
  this->sockets_->open();

  std::vector<std::string> joint_names(7);
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    joint_names.at(j) = prefix + "joint" + std::to_string(j + 1);
  }
  this->state_ = JointState("franka", joint_names);
  this->last_command_ = std::chrono::steady_clock::now();
}

void FrankaLightWeightInterface::set_joint_damping(const Eigen::Array<double, 7, 1>& joint_damping_gains) {
  this->joint_damping_gains_ = joint_damping_gains;
}

void FrankaLightWeightInterface::set_joint_damping(const std::array<double, 7>& joint_damping_gains) {
  this->set_joint_damping(Eigen::ArrayXd::Map(joint_damping_gains.data(), 7));
}

void FrankaLightWeightInterface::set_joint_impedance(const Eigen::Array<double, 7, 1>& joint_impedance_values) {
  std::array<double, 7> values{};
  for (std::size_t i = 0; i < 7; ++i) {
    values.at(i) = joint_impedance_values(i);
  }
  this->set_joint_impedance(values);
}

void FrankaLightWeightInterface::set_joint_impedance(const std::array<double, 7>& joint_impedance_values) {
  this->joint_impedance_values_ = joint_impedance_values;
}

void FrankaLightWeightInterface::set_collision_behaviour(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal) {
  this->set_collision_behaviour(
      {lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
       upper_torque_thresholds_nominal, lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
       lower_force_thresholds_nominal, upper_force_thresholds_nominal});
}

void FrankaLightWeightInterface::set_collision_behaviour(const CollisionBehaviour& collision_behaviour) {
  this->collision_behaviour_ = collision_behaviour;
}

void FrankaLightWeightInterface::run_controller() {
  if (this->connected_) {
    // restart the controller unless the node is shutdown
    while (!this->shutdown_) {
      try {
        if (this->command_ == nullptr) {
          std::cout << "Starting state publisher..." << std::endl;
          this->run_state_publisher();
        } else {
          if (this->command_->get_type() == StateType::JOINT_VELOCITIES) {
            std::cout << "Starting joint velocity controller..." << std::endl;
            this->run_joint_velocities_controller();
          } else if (this->command_->get_type() == StateType::JOINT_TORQUES) {
            std::cout << "Starting joint torque controller..." << std::endl;
            this->run_joint_torques_controller();
          } else {
            std::cout << "Unimplemented control type!" << std::endl;
            std::cout << "Starting state publisher..." << std::endl;
            this->run_state_publisher();
          }
        }
      } catch (const franka::CommandException& e) {
        std::cerr << e.what() << std::endl;
      }
      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
      this->command_.reset();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    throw std::runtime_error("Robot not connected! Call the init function first.");
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  std::string msg;
  if (this->sockets_->receive_bytes(msg)) {
    std::shared_ptr<JointState> state;
    auto command_type = clproto::check_message_type(msg);
    if (command_type == clproto::MessageType::JOINT_VELOCITIES_MESSAGE) {
      state = std::make_shared<JointVelocities>(clproto::decode<JointVelocities>(msg));
    } else if (command_type == clproto::MessageType::JOINT_TORQUES_MESSAGE) {
      state = std::make_shared<JointTorques>(clproto::decode<JointTorques>(msg));
    }
    if (state == nullptr) {
      return;
    }
    this->last_command_ = std::chrono::steady_clock::now();
    this->command_ = state;
  } else if (
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->last_command_)
          .count()
      > this->command_timeout_.count()) {
    this->command_.reset();
  }
}

void FrankaLightWeightInterface::read_and_publish_robot_state(const franka::RobotState& robot_state) {
  assert(robot_state.q.size() == 7);
  this->state_.set_positions(Eigen::VectorXd::Map(robot_state.q.data(), 7));
  this->state_.set_velocities(Eigen::VectorXd::Map(robot_state.dq.data(), 7));
  this->state_.set_torques(Eigen::VectorXd::Map(robot_state.tau_J.data(), 7));

  std::string state_msg = clproto::encode(this->state_);
  this->sockets_->send_bytes(state_msg);
}

void FrankaLightWeightInterface::run_state_publisher() {
  try {
    this->franka_robot_->read([this](const franka::RobotState& robot_state) {
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->read_and_publish_robot_state(robot_state);
      }
      this->poll_external_command();
      if (this->command_
          && (this->command_->get_type() == StateType::JOINT_VELOCITIES
              || this->command_->get_type() == StateType::JOINT_TORQUES)) {
        std::cout << "Received a new control type command - switching! " << std::endl;
        return false;
      }
      return true;
    });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_joint_velocities_controller() {
  this->franka_robot_->setJointImpedance(this->joint_impedance_values_);

  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  try {
    this->franka_robot_->control(
        [this](const franka::RobotState& robot_state, franka::Duration) -> franka::JointVelocities {
          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->read_and_publish_robot_state(robot_state);
          }
          this->poll_external_command();
          if (this->command_ == nullptr) {
            throw franka::ControlException("Control type reset!");
          }
          if (this->command_->get_type() != StateType::JOINT_VELOCITIES) {
            throw std::runtime_error("Control type changed!");
          }

          std::array<double, 7> velocities{};
          Eigen::VectorXd::Map(&velocities[0], 7) = this->command_->get_velocities().array();
          return velocities;
        });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_joint_torques_controller() {
  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  try {
    this->franka_robot_->control([this](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->read_and_publish_robot_state(robot_state);
      }
      this->poll_external_command();

      if (this->command_ == nullptr) {
        throw franka::ControlException("Control type reset!");
      }
      if (this->command_->get_type() != StateType::JOINT_TORQUES) {
        throw std::runtime_error("Control type changed!");
      }

      std::array<double, 7> coriolis_array = this->franka_model_->coriolis(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

      std::array<double, 49> mass_array = franka_model_->mass(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

      std::array<double, 7> torques{};
      Eigen::VectorXd::Map(&torques[0], 7) = this->command_->get_torques().array()
          - this->joint_damping_gains_ * this->state_.get_velocities().array() + coriolis.array();
      return torques;
    });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}// namespace frankalwi
