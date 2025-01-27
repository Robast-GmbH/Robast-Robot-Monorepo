#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class ZedMock : public rclcpp::Node
{
  public:
    ZedMock() : Node("zed_mock")
    {
      person_detection_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/person_detection", 10);
      timer_callback_ =
        this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ZedMock::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::PointStamped();
      message.header.frame_id = "";
      message.point.x = 1.0;
      message.point.y = 0.0;
      message.point.z = 0.0;
      person_detection_publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr person_detection_publisher_;
    rclcpp::TimerBase::SharedPtr timer_callback_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedMock>());
  rclcpp::shutdown();
  return 0;
}
