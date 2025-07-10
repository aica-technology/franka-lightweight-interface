#include <clproto.hpp>
#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

int main(int argc, char** argv) {
  communication_interfaces::sockets::ZMQCombinedSocketsConfiguration zmq_config;
  zmq_config.context = std::make_shared<::zmq::context_t>(1);
  zmq_config.ip_address = "*";
  zmq_config.publisher_port = "1601";
  zmq_config.subscriber_port = "1602";
  zmq_config.bind_publisher = true;
  zmq_config.bind_subscriber = true;
  if (argc == 2) {
    if (atof(argv[1]) == 17) {
      zmq_config.publisher_port = "1701";
      zmq_config.subscriber_port = "1702";
    } else if (atof(argv[1]) != 16) {
      std::cerr << "This robot is unknown, choose either '16' (default) or '17'." << std::endl;
      return 1;
    }
  } else {
    std::cerr << "Please provide at most one argument to choose which robot to connect to ('16' or '17')." << std::endl;
    return 1;
  }

  double gain = 0.5;

  communication_interfaces::sockets::ZMQPublisherSubscriber sockets(zmq_config);
  sockets.open();

  state_representation::JointPositions target;
  while (zmq_config.context->handle() != nullptr) {
    std::string msg;
    if (sockets.receive_bytes(msg)) {
      auto joint_state = clproto::decode<state_representation::JointState>(msg);
      if (target.is_empty()) {
        target = joint_state;
      }

      auto velocities = gain * (target.get_positions() - joint_state.get_positions());
      auto command = state_representation::JointVelocities("franka", joint_state.get_names(), velocities);

      auto send_msg = clproto::encode(command);
      sockets.send_bytes(send_msg);
    }
  }
}
