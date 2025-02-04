#include "qr_code_scanner.hpp"

QrCodeScanner::QrCodeScanner() : Node("qr_code_scanner")
{
  auto image_subscription_options = SubscriptionOptions();
  image_subscription_options.callback_group = this->create_callback_group(CallbackGroupType::Reentrant);
  _zed_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/robot/zed/zed_node/left_gray/image_rect_gray/compressed",
    10,
    bind(&QrCodeScanner::image_callback, this, placeholders::_1),
    image_subscription_options);

  _read_qr_code_service = this->create_service<communication_interfaces::srv::ReadQrCode>(
    "read_qr_code", bind(&QrCodeScanner::read_qr_code_service_callback, this, placeholders::_1, placeholders::_2));
}

QrCodeScanner::~QrCodeScanner()
{
  quirc_destroy(_qr_decoder);
}

void QrCodeScanner::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "image received");
  if (_active_service_calls_counter <= 0)
  {
    return;
  }
  cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  read_qr_codes_from_image(image, visible_qr_codes);
  for (auto qrCode : visible_qr_codes)
  {
    RCLCPP_INFO(this->get_logger(), "QR Code: %s", qrCode.c_str());
  }
}

void QrCodeScanner::read_qr_code_service_callback(
  const shared_ptr<communication_interfaces::srv::ReadQrCode::Request> request,
  shared_ptr<communication_interfaces::srv::ReadQrCode::Response> response)
{
  visible_qr_codes.clear();
  _active_service_calls_counter++;
  bool found_match = false;

  int timeout_counter = 0;
  int timeout_limit = request->timeout_in_s * 10;

  while (!found_match && timeout_counter < timeout_limit)
  {
    found_match =
      find(visible_qr_codes.begin(), visible_qr_codes.end(), request->expected_data) != visible_qr_codes.end();

    this_thread::sleep_for(chrono::milliseconds(100));
    timeout_counter++;
  }

  _active_service_calls_counter--;
  response->success = found_match;
}

void QrCodeScanner::read_qr_codes_from_image(cv::Mat &image, vector<string> &qr_codes)
{
  qr_codes.clear();
  int width = image.cols;
  int height = image.rows;

  quirc_resize(_qr_decoder, width, height);

  // Convert image to grayscale
  cv::Mat grey_mat;
  cvtColor(image, grey_mat, cv::COLOR_BGR2GRAY);

  // Prepare Quirc to decode the QR code
  uint8_t *qr_image = quirc_begin(_qr_decoder, NULL, NULL);
  memcpy(qr_image, grey_mat.data, width * height);
  quirc_end(_qr_decoder);

  // Detect and decode QR codes
  int num_codes = quirc_count(_qr_decoder);
  for (int i = 0; i < num_codes; i++)
  {
    struct quirc_code code;
    quirc_extract(_qr_decoder, i, &code);

    // Decode the QR code
    struct quirc_data data;
    quirc_decode_error_t err = quirc_decode(&code, &data);
    if (!err)
    {
      qr_codes.push_back((char *) data.payload);
    }
  }
}
