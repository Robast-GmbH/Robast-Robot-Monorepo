#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SoundPlayerNode : public rclcpp::Node
{
 public:
  SoundPlayerNode() : Node("sound_player_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/sound", 10, std::bind(&SoundPlayerNode::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());

    // Play the sound file using mpg123 with PulseAudio
    std::string command =
        "mpg123 -o pulse "
        "/workspace/sounds/"
        "ElevenLabs_2025-01-08T11_01_51_Rachel_pre_s50_sb75_se0_b_m2.mp3";
    int result = std::system(command.c_str());
    if (result != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to play sound file");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SoundPlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}