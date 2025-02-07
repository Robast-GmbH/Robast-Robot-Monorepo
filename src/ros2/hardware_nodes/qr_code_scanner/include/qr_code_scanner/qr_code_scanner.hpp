#include <condition_variable>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <thread>
#include <vector>

#include "communication_interfaces/srv/read_qr_code.hpp"
#include "quirc.h"
#include "rclcpp/rclcpp.hpp"

class QrCodeScanner : public rclcpp::Node
{
  public:
    QrCodeScanner();
    ~QrCodeScanner();

  private:
    void toggle_image_subscription(bool enable);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void read_qr_code_service_callback(
      const std::shared_ptr<communication_interfaces::srv::ReadQrCode::Request> request,
      std::shared_ptr<communication_interfaces::srv::ReadQrCode::Response> response);
    std::vector<std::string> read_qr_codes_from_image(const cv::Mat &image);

    int _active_service_calls_counter = 0;
    struct quirc *_qr_decoder = quirc_new();
    std::vector<std::string> _visible_qr_codes;
    rclcpp::SubscriptionOptions image_subscription_options;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _zed_image_subscriber;
    rclcpp::Service<communication_interfaces::srv::ReadQrCode>::SharedPtr _read_qr_code_service;
};