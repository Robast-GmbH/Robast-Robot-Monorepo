#ifndef CAMERA_SUBSCRIBER_HPP
#define CAMERA_SUBSCRIBER_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

class CameraSubscriber : public rclcpp::Node
{
public:
  CameraSubscriber();

private:
  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
  
  void processImage(cv::Mat& img);
  void runInference(cv::Mat& img);
  void visualizeDetections(cv::Mat& img);
};

#endif 