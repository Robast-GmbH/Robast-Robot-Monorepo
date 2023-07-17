#include "door_handle_dectection_package/door_handle_detection_sim.hpp"

CameraSubscriber::CameraSubscriber()
  : Node("camera_subscriber")
{
  // Create the image subscriber
  image_transport::ImageTransport it(this);
  sub_ = it.subscribe("rgb_cam/image_raw", 10, &CameraSubscriber::cameraCallback, this);
}

void CameraSubscriber::cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img = cv_ptr->image;

  processImage(img);
}

void CameraSubscriber::processImage(cv::Mat& img)
{
  runInference(img);
  visualizeDetections(img);

  cv::imshow("IMAGE", img);
  cv::waitKey(4);
}

void CameraSubscriber::runInference(cv::Mat& img)
{
  // Perform inference on the input image using your model
  // ...
  // Replace this placeholder logic with your model inference logic
  // ...
}

void CameraSubscriber::visualizeDetections(cv::Mat& img)
{
  // Visualize the detected objects on the image
  // ...
  // Replace this placeholder logic with your visualization logic
  // ...
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto camera_subscriber = std::make_shared<CameraSubscriber>();
  rclcpp::spin(camera_subscriber);
  rclcpp::shutdown();
  return 0;
}
