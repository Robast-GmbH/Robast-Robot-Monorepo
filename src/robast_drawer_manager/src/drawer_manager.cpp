#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;



class DrawerManager : public rclcpp::Node
{
  public:
    DrawerManager()
    : Node("drawer_manager"), count_(0)
    {

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = Node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    }

  private:
    void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)


}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawerManager>());
  rclcpp::shutdown();
  return 0;
}