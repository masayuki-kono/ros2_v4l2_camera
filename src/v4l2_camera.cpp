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

#include "v4l2_camera/v4l2_camera.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "v4l2_camera/fourcc.hpp"
#include "v4l2_camera/parameters.hpp"

using namespace std::chrono_literals;

namespace v4l2_camera
{

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  parameters_{get_node_parameters_interface(), get_node_topics_interface(),
  get_node_logging_interface()},
  subscribers_count_{0}
{
  parameters_.declareStaticParameters();
  parameters_.declareOutputParameters();

  // Prepare camera
  camera_ = std::make_shared<V4l2CameraDevice>(
    parameters_.getVideoDevice(),
    parameters_.getValue<double>("capture_timeout"));

  if (!camera_->open()) {
    return;
  }

  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  parameters_.declareDeviceParameters(*camera_);

  // Read parameters and set up callback
  applyParameters();

  parameters_.setParameterChangedCallback(
    [this](rclcpp::Parameter parameter) {
      handleParameter(parameter);
    });

  auto image_topic_name = std::string(get_name()) + "/image_raw";

  // Allow overriding QoS settings (history, depth, reliability)
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  image_pub_ = image_transport::create_camera_publisher(this,
    image_topic_name, rmw_qos_profile_default, pub_options);

  if (camera_->start()) {
    startStreamingTimer();
  }
  else {
    startReconnectTimer();
  }
}

V4L2Camera::~V4L2Camera()
{
  if (reconnect_timer_) {
    reconnect_timer_->reset();
  }
  if (streaming_timer_) {
    streaming_timer_->reset();
  }
  if (camera_) {
    camera_->stop();
  }
}

void V4L2Camera::applyParameters()
{
  output_encoding_ = parameters_.getOutputEncoding();

  // Camera info parameters
  auto camera_info_url = parameters_.getCameraInfoUrl();
  if (camera_info_url != "") {
    if (camera_info_->validateURL(camera_info_url)) {
      camera_info_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  camera_frame_id_ = parameters_.getCameraFrameId();

  // Format parameters
  // Pixel format
  auto pixel_format = parameters_.getPixelFormat();
  requestPixelFormat(pixel_format);

  // Image size
  auto image_size = parameters_.getImageSize();
  requestImageSize(image_size);

  // Control parameters
  auto control_parameters = parameters_.getControlParameters();
  for (auto const & param : control_parameters) {
    auto control_id = parameters_.getControlId(param);
    auto control = camera_->queryControl(control_id);
    if (control.inactive) {
      RCLCPP_DEBUG(get_logger(), "Skipping inactive control: %s", control.name.c_str());
      continue;
    }

    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (static_cast<bool>(camera_->getControlValue(control.id)) == param.as_bool()) {continue;}
        camera_->setControlValue(control_id, param.as_bool());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (camera_->getControlValue(control.id) == param.as_int()) {continue;}
        camera_->setControlValue(control_id, param.as_int());
        break;
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %d, for parameter: %s",
          unsigned(param.get_type()), param.get_name().c_str());
    }
  }
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = param.get_name();
  if (parameters_.isControlParameter(param)) {
    auto control_id = parameters_.getControlId(param);
    auto control = camera_->queryControl(control_id);
    if (control.inactive) {
      RCLCPP_WARN(get_logger(), "Cannot set inactive control: %s", control.name.c_str());
      return false;
    }
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (static_cast<bool>(camera_->getControlValue(control.id)) == param.as_bool()) {
          RCLCPP_DEBUG(
            get_logger(), "Parameter %s already set at requested value: %d",
            control.name.c_str(), param.as_bool());
          return true;
        }
        return camera_->setControlValue(control_id, param.as_bool());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (camera_->getControlValue(control.id) == param.as_int()) {
          RCLCPP_DEBUG(
            get_logger(), "Parameter %s already set at requested value: %ld",
            control.name.c_str(), param.as_int());
          return true;
        }
        return camera_->setControlValue(control_id, param.as_int());
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %s, for parameter: %s",
          std::to_string(unsigned(param.get_type())).c_str(), param.get_name().c_str());
    }
  } else if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "pixel_format") {
    camera_->stop();
    auto success = requestPixelFormat(param.as_string());
    camera_->start();
    return success;
  } else if (param.get_name() == "image_size") {
    camera_->stop();
    auto success = requestImageSize(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "camera_info_url") {
    auto camera_info_url = param.as_string();
    if (camera_info_->validateURL(camera_info_url)) {
      return camera_info_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      return false;
    }
  }

  return false;
}

bool V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Invalid pixel format size: must be a 4 character code (FOURCC).");
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid image size; expected dimensions: 2, actual: %lu",
      size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

void V4L2Camera::startReconnectTimer()
{
  reconnect_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(parameters_.getReconnectInterval() * 1000)),
    [this]() {
      if (camera_->start()) {
        RCLCPP_INFO(get_logger(), "Reconnected to camera, starting streaming");
        reconnect_timer_.reset();
        startStreamingTimer();
      }
      else {
        RCLCPP_ERROR(get_logger(), "Failed to reconnect to camera, retrying...");
      }
    });
}

void V4L2Camera::startStreamingTimer()
{
  streaming_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.getFps())),
    std::bind(&V4L2Camera::streamingTimerCallback, this));
}

void V4L2Camera::streamingTimerCallback()
{
  auto previous_subscribers_count = subscribers_count_;
  subscribers_count_ = image_pub_.getNumSubscribers();
  if (subscribers_count_ == 0) {
    if (previous_subscribers_count > 0) {
      RCLCPP_INFO(get_logger(), "No subscriber, stopping to capture");
    }
    return;
  }
  else {
    if (previous_subscribers_count == 0) {
      RCLCPP_INFO(get_logger(), "Detect subscriber, starting to capture");
    }
  }

  auto img = camera_->capture();
  if (!img) {
    RCLCPP_ERROR(get_logger(), "Failed to capture image, reconnecting...");
    camera_->stop();
    streaming_timer_.reset();
    startReconnectTimer();
    return;
  }

  auto stamp = now();
  img->header.stamp = stamp;
  img->header.frame_id = camera_frame_id_;

  auto cvImg = cv_bridge::toCvCopy(*img);

  if (img->encoding != output_encoding_) {
    cvImg = cv_bridge::cvtColor(cvImg, output_encoding_);
  }

  int rotateFlag = parameters_.getRotateFlag();
  if (rotateFlag >= 0 && rotateFlag <= 2) {
    cv::Mat rotatedImg;
    cv::rotate(cvImg->image, rotatedImg, rotateFlag);
    cvImg->image = rotatedImg;
  }

  int flipCode = parameters_.getFlipCode();
  if (flipCode >= -1 && flipCode <= 1) {
    cv::Mat flippedImg;
    cv::flip(cvImg->image, flippedImg, flipCode);
    cvImg->image = flippedImg;
  }

  cvImg->toImageMsg(*img);

  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_->getCameraInfo());
  if (!checkCameraInfo(*img, *ci)) {
    *ci = sensor_msgs::msg::CameraInfo{};
    ci->height = img->height;
    ci->width = img->width;
  }
  ci->header.stamp = stamp;

  image_pub_.publish(std::move(img), std::move(ci));
}

}  // namespace v4l2_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
