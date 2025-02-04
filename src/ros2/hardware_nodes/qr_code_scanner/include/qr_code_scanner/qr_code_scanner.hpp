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

using namespace std;
using namespace rclcpp;

class QrCodeScanner : public rclcpp::Node
{
  public:
    QrCodeScanner();
    ~QrCodeScanner();

  private:
    void read_qr_codes_from_image(const cv::Mat &image, vector<std::string> &qr_codes);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void read_qr_code_service_callback(const shared_ptr<communication_interfaces::srv::ReadQrCode::Request> request,
                                       shared_ptr<communication_interfaces::srv::ReadQrCode::Response> response);

    int _active_service_calls_counter = 1;
    struct quirc *_qr_decoder = quirc_new();
    vector<std::string> visible_qr_codes;
    Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _zed_image_subscriber;
    Service<communication_interfaces::srv::ReadQrCode>::SharedPtr _read_qr_code_service;
};