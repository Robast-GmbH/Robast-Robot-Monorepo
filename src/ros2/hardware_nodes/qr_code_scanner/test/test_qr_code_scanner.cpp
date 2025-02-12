#include <catch2/catch_all.hpp>

#include "qr_code_scanner.hpp"

using namespace std;
using namespace rclcpp;

class ImagePublisher : public rclcpp::Node
{
  public:
    explicit ImagePublisher(string image_path) : Node("image_publisher_node")
    {
      auto _image = cv::imread(image_path, cv::IMREAD_COLOR);
      REQUIRE(!_image.empty());
      _image_data = image_to_uchar_vector(_image);

      auto pub_options = rclcpp::PublisherOptions();
      pub_options.callback_group = this->create_callback_group(CallbackGroupType::Reentrant);
      _image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/robot/zed/zed_node/left_gray/image_rect_gray/compressed", 10, pub_options);
      _timer = this->create_wall_timer(100ms,
                                       [this]()
                                       {
                                         this->publish_image();
                                       });
    }

  private:
    std::vector<uchar> image_to_uchar_vector(const cv::Mat &image)
    {
      std::vector<uchar> compressed_image_data;
      std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};   // JPEG quality setting (0-100)
      bool success = cv::imencode(".jpg", image, compressed_image_data, compression_params);
      REQUIRE(success);
      return compressed_image_data;
    }

    void publish_image()
    {
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = this->now();
      msg.format = "jpeg";
      msg.data = _image_data;

      _image_publisher->publish(msg);
    };

    std::vector<uchar> _image_data;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _image_publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

TEST_CASE("TestQrCodeScanner")
{
  // Setup
  const string expected_qr_code = "http://en.m.wikipedia.org";
  const string example_path = "/workspace/src/hardware_nodes/qr_code_scanner/test/qr_code.png";

  rclcpp::init(0, nullptr);

  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  auto qr_code_scanner = std::make_shared<QrCodeScanner>();
  auto image_publisher = std::make_shared<ImagePublisher>(example_path);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(image_publisher);
  executor.add_node(qr_code_scanner);

  std::jthread main_thread(
    [&executor]()
    {
      executor.spin();
    });

  auto client = test_node->create_client<communication_interfaces::srv::ReadQrCode>("/read_qr_code");

  // Run
  if (!client->wait_for_service(1s))
  {
    FAIL("Service not available");
  }

  auto request = std::make_shared<communication_interfaces::srv::ReadQrCode::Request>();
  request->expected_data = expected_qr_code;
  request->timeout_in_s = 5;

  auto response_future = client->async_send_request(request);
  response_future.wait_for(std::chrono::seconds(2));

  auto response = std::make_shared<communication_interfaces::srv::ReadQrCode::Response>();
  response = response_future.get();

  REQUIRE(response->success == true);

  // Tear down
  rclcpp::shutdown();
  if (main_thread.joinable())
  {
    main_thread.join();
  }
}
