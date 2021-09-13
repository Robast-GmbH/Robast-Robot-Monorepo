#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "task_handler.cpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

std::mutex myMutex;
 

class TaskBuilder : public rclcpp::Node
{   
  private:
    TaskHandler* handler;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
    
  public:
    TaskBuilder()
    : Node("task_builder"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("task", 10);
      subscription_ = this->create_subscription<std_msgs::msg::String>("robo_task", 10, std::bind(&TaskBuilder::handle_task, this, _1));
      handler = new TaskHandler();
    }
    
    void handle_task(const std_msgs::msg::String::SharedPtr msg) const
    {  
      
     Task input=  handler->rosToTask(std_msgs::msg::String());
      //is POI? (or Task)
      if(true)
      {
        handler->addPoi(input);
      }
      else
      {
        Task  roboTask = handler->buildTask(&input);
        std_msgs::msg::String message =  handler->TaskToRos(&roboTask);
        publisher_->publish(message);
      }
      RCLCPP_INFO(this->get_logger(), "Message send '%s'", msg->data.c_str());
    }
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskBuilder>());
  rclcpp::shutdown();
  return 0;
}