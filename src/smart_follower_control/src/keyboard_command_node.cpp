#include <chrono>
#include <cctype>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace smart_follower_control
{

class KeyboardCommandNode : public rclcpp::Node
{
public:
  KeyboardCommandNode()
  : Node("keyboard_command_node")
  {
    declare_parameter("follow_command_topic", std::string("follow_command"));
    declare_parameter("key_lock", std::string("l"));
    declare_parameter("key_unlock", std::string("u"));
    declare_parameter("key_reset", std::string("r"));
    declare_parameter("key_estop", std::string("q"));

    topic_ = get_parameter("follow_command_topic").as_string();
    key_lock_ = key_from_param("key_lock", 'l');
    key_unlock_ = key_from_param("key_unlock", 'u');
    key_reset_ = key_from_param("key_reset", 'r');
    key_estop_ = key_from_param("key_estop", 'q');

    pub_ = create_publisher<smart_follower_msgs::msg::FollowCommand>(topic_, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&KeyboardCommandNode::poll_key, this));

#ifndef _WIN32
    setup_terminal();
#endif

    RCLCPP_INFO(get_logger(), "Keyboard command node ready. Keys: L lock, U unlock, R reset, Q estop");
  }

  ~KeyboardCommandNode() override
  {
#ifndef _WIN32
    restore_terminal();
#endif
  }

private:
  std::string topic_;
  char key_lock_{'l'};
  char key_unlock_{'u'};
  char key_reset_{'r'};
  char key_estop_{'q'};

  rclcpp::Publisher<smart_follower_msgs::msg::FollowCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

#ifndef _WIN32
  struct termios old_termios_{};
  bool termios_ready_{false};
#endif

  char key_from_param(const std::string & name, char fallback)
  {
    const auto s = get_parameter(name).as_string();
    if (s.empty()) {
      return fallback;
    }
    return static_cast<char>(std::tolower(static_cast<unsigned char>(s[0])));
  }

#ifndef _WIN32
  void setup_terminal()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }
    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      return;
    }
    termios newt = old_termios_;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    termios_ready_ = true;
  }

  void restore_terminal()
  {
    if (termios_ready_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      termios_ready_ = false;
    }
  }
#endif

  int read_key()
  {
#ifndef _WIN32
    unsigned char c;
    const int n = static_cast<int>(::read(STDIN_FILENO, &c, 1));
    if (n == 1) {
      return std::tolower(c);
    }
    return -1;
#else
    return -1;
#endif
  }

  void publish_cmd(uint8_t cmd)
  {
    smart_follower_msgs::msg::FollowCommand msg;
    msg.header.stamp = now();
    msg.command = cmd;
    msg.target_id = -1;
    pub_->publish(msg);
  }

  void poll_key()
  {
    const int key = read_key();
    if (key < 0) {
      return;
    }

    if (key == key_lock_) {
      publish_cmd(smart_follower_msgs::msg::FollowCommand::LOCK);
      RCLCPP_INFO(get_logger(), "LOCK command");
    } else if (key == key_unlock_) {
      publish_cmd(smart_follower_msgs::msg::FollowCommand::UNLOCK);
      RCLCPP_INFO(get_logger(), "UNLOCK command");
    } else if (key == key_reset_) {
      publish_cmd(smart_follower_msgs::msg::FollowCommand::RESET);
      RCLCPP_INFO(get_logger(), "RESET command");
    } else if (key == key_estop_) {
      publish_cmd(smart_follower_msgs::msg::FollowCommand::ESTOP);
      RCLCPP_INFO(get_logger(), "ESTOP command");
    }
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smart_follower_control::KeyboardCommandNode>());
  rclcpp::shutdown();
  return 0;
}
