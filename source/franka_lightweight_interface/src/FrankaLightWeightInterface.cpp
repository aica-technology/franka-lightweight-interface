#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

#include <stdexcept>
#include <thread>

#include <clproto.hpp>
#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>

using namespace state_representation;

class IncompatibleControlTypeException : public std::runtime_error {
public:
  explicit IncompatibleControlTypeException(const std::string& msg) : runtime_error(msg){};
};

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

FrankaLightWeightInterface::FrankaLightWeightInterface(
    std::string robot_ip, communication_interfaces::sockets::ZMQCombinedSocketsConfiguration zmq_config,
    std::string prefix)
    : prefix_(std::move(prefix)),
      robot_ip_(std::move(robot_ip)),
      connected_(false),
      shutdown_(false),
      sockets_(zmq_config),
      joint_damping_gains_(default_joint_damping_gains()),
      joint_impedance_values_(default_joint_impedance_values()),
      collision_behaviour_(default_collision_behaviour()) {}

void FrankaLightWeightInterface::init() {
  // create connection to the robot
  this->franka_robot_ = std::make_unique<franka::Robot>(this->robot_ip_);
  this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());
  this->connected_ = true;

  // create zmq connections with an external controller
  // TODO: find a better way to pass in port number
  sockets_.open();

  if (this->prefix_.empty()) {
    this->prefix_ = "franka_";
  }
  std::string robot_name = this->prefix_.substr(0, this->prefix_.length() - 1);
  std::vector<std::string> joint_names(7);
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    joint_names.at(j) = this->prefix_ + "joint" + std::to_string(j + 1);
  }
  this->state_.ee_state = CartesianState(this->prefix_ + "ee", this->prefix_ + "base");
  this->state_.joint_state = JointState(robot_name, joint_names);
  // this->state_.jacobian =
  //     state_representation::Jacobian(robot_name, joint_names, this->prefix_ + "ee", this->prefix_ + "base");
  // this->state_.mass =
  //     state_representation::Parameter<Eigen::MatrixXd>(this->prefix_ + "mass", Eigen::MatrixXd::Zero(7, 7));

  this->last_command_ = std::chrono::steady_clock::now();
}

void FrankaLightWeightInterface::reset_command() {
  this->command_.reset();
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
  if (this->is_connected()) {
    // restart the controller unless the node is shutdown
    while (!this->is_shutdown()) {
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
      //flush and reset any remaining command messages
      // network_interfaces::zmq::receive(this->command_, this->zmq_subscriber_);
      this->reset_command();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    throw std::runtime_error("Robot not connected! Call the init function first.");
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  std::string msg;
  if (this->sockets_.receive_bytes(msg)) {
    std::shared_ptr<JointState> state;
    if (auto command_type = clproto::check_message_type(msg);
        command_type == clproto::MessageType::JOINT_VELOCITIES_MESSAGE) {
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
    this->reset_command();
  }
}

void FrankaLightWeightInterface::publish_robot_state() {
  std::vector<std::string> encoded_state;
  encoded_state.emplace_back(clproto::encode(this->state_.ee_state));
  encoded_state.emplace_back(clproto::encode(this->state_.joint_state));
  std::string msg;
  clproto::pack_fields(encoded_state, msg.data());
  this->sockets_.send_bytes(msg);
}

void FrankaLightWeightInterface::read_robot_state(const franka::RobotState& robot_state) {
  // extract cartesian info
  Eigen::Affine3d eef_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  this->state_.ee_state.set_pose(eef_transform.translation(), Eigen::Quaterniond(eef_transform.linear()));
  this->state_.ee_state.set_wrench(Eigen::MatrixXd::Map(robot_state.O_F_ext_hat_K.data(), 6, 1));

  // extract joint info
  assert(robot_state.q.size() == 7);
  this->state_.joint_state.set_positions(Eigen::VectorXd::Map(robot_state.q.data(), 7));
  this->state_.joint_state.set_velocities(Eigen::VectorXd::Map(robot_state.dq.data(), 7));
  this->state_.joint_state.set_torques(Eigen::VectorXd::Map(robot_state.tau_J.data(), 7));

  // extract jacobian
  std::array<double, 42> jacobian_array = this->franka_model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  this->state_.jacobian.set_data(Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));

  // std::array<double, 49> current_mass_array = this->franka_model_->mass(robot_state);
  // this->state_.mass.set_value(Eigen::Map<const Eigen::Matrix<double, 7, 7>>(current_mass_array.data()));

  // get the twist from jacobian and current joint velocities
  this->state_.ee_state.set_twist(this->state_.jacobian * this->state_.joint_state.get_velocities());
}

void FrankaLightWeightInterface::run_state_publisher() {
  try {
    this->franka_robot_->read([this](const franka::RobotState& robot_state) {
      // check the local socket for a command
      this->poll_external_command();
      if (this->command_) {
        std::cout << "Received a new control type command - switching! " << std::endl;
        return false;
      }
      this->read_robot_state(robot_state);
      this->publish_robot_state();
      return true;
    });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_joint_velocities_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  this->franka_robot_->setJointImpedance(this->joint_impedance_values_);

  // Set collision behavior.
  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  try {
    this->franka_robot_->control(
        [this](const franka::RobotState& robot_state, franka::Duration) -> franka::JointVelocities {
          // check the local socket for a velocity command
          this->poll_external_command();

          if (this->command_ == nullptr) {
            throw franka::ControlException("Control type reset!");
          }
          if (this->command_->get_type() != StateType::JOINT_VELOCITIES) {
            throw IncompatibleControlTypeException("Control type changed!");
          }

          // lock mutex
          std::lock_guard<std::mutex> lock(this->get_mutex());
          // extract current state
          this->read_robot_state(robot_state);

          std::array<double, 7> velocities{};
          Eigen::VectorXd::Map(&velocities[0], 7) = this->command_->get_velocities().array();

          // write the state out to the local socket
          this->publish_robot_state();

          //return velocities;
          return velocities;
        });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_joint_torques_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  // Set collision behavior.
  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  try {
    this->franka_robot_->control([this](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
      // check the local socket for a torque command
      this->poll_external_command();

      if (this->command_ == nullptr) {
        throw franka::ControlException("Control type reset!");
      }
      if (this->command_->get_type() != StateType::JOINT_TORQUES) {
        throw IncompatibleControlTypeException("Control type changed!");
      }

      // lock mutex
      std::lock_guard<std::mutex> lock(this->get_mutex());
      // extract current state
      this->read_robot_state(robot_state);

      // get the coriolis array
      std::array<double, 7> coriolis_array = this->franka_model_->coriolis(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

      // get the mass matrix
      std::array<double, 49> mass_array = franka_model_->mass(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

      std::array<double, 7> torques{};
      Eigen::VectorXd::Map(&torques[0], 7) = this->command_->get_torques().array()
          - this->joint_damping_gains_ * this->state_.joint_state.get_velocities().array() + coriolis.array();

      // write the state out to the local socket
      this->publish_robot_state();

      //return torques;
      return torques;
    });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}// namespace frankalwi
