// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <memory>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "v4l2_camera/visibility_control.h"
#include "v4l2_camera/parameters.hpp"

namespace v4l2_camera
{

class V4L2Camera : public rclcpp::Node
{
public:
  explicit V4L2Camera(rclcpp::NodeOptions const & options);

  virtual ~V4L2Camera();

private:
  std::shared_ptr<V4l2CameraDevice> camera_;
  image_transport::CameraPublisher image_pub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr streaming_timer_;
  Parameters parameters_;
  bool device_parameters_declared_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_;

  std::size_t subscribers_count_;
  std::string camera_frame_id_;
  std::string output_encoding_;
  int last_exposure_time_absolute_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  void applyParameters();
  bool handleParameter(rclcpp::Parameter const & param);

  bool requestPixelFormat(std::string const & fourcc);
  bool requestImageSize(std::vector<int64_t> const & size);

  bool checkCameraInfo(
    sensor_msgs::msg::Image const & img,
    sensor_msgs::msg::CameraInfo const & ci);

  void startReconnectTimer();
  void startStreamingTimer();
  void streamingTimerCallback();
  int calculateExposureTime(int prev_exposure_time, const cv::Mat& image);
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_HPP_
