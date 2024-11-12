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

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
  streaming_enabled_{false}
{
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters
  auto image_topic_name = std::string(get_name()) + "/image_raw";
  auto info_topic_name = std::string(get_name()) + "/camera_info";
  if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_name, 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, 10);
  } else {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, image_topic_name);
  }

  parameters_.declareStaticParameters();
  parameters_.declareOutputParameters();

  // Prepare camera
  camera_ = std::make_shared<V4l2CameraDevice>(
    parameters_.getVideoDevice(),
    get_node_logging_interface());

  if (!camera_->open()) {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  parameters_.declareDeviceParameters(*camera_);

  // Read parameters and set up callback
  applyParameters();

  parameters_.setParameterChangedCallback(
    [this](rclcpp::Parameter parameter) {
      handleParameter(parameter);
    });

  if (!camera_->start()) {
    return;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  {
    auto callback = [this](const sun_msgs::msg::OperatingStatus::SharedPtr msg) {
      streaming_enabled_ = msg->state == sun_msgs::msg::OperatingStatus::MANUAL_OPERATING;
      if (streaming_enabled_) {
        if (!streaming_timer_) {
          std::chrono::milliseconds period(static_cast<int>(1000.0 / parameters_.getFps()));
          streaming_timer_ = create_wall_timer(period, [this]() {
            capture_and_publish(); });
        }
      }
      else {
        if (streaming_timer_) {
          streaming_timer_->cancel();
          streaming_timer_.reset();
        }
      }
    };
    op_status_sub_ = create_subscription<sun_msgs::msg::OperatingStatus>(
      "/operating/status", qos, callback);
  }
}

V4L2Camera::~V4L2Camera()
{
    camera_->stop();
}

void V4L2Camera::applyParameters()
{
  output_encoding_ = parameters_.getOutputEncoding();

  // Camera info parameters
  auto camera_info_url = parameters_.getCameraInfoUrl();
  if (camera_info_url != "") {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
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
    if (cinfo_->validateURL(camera_info_url)) {
      return cinfo_->loadCameraInfo(camera_info_url);
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

sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  auto cvImg = cv_bridge::toCvCopy(img);

  if (img.encoding != output_encoding_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Image encoding not the same as requested output, performing possibly slow conversion: "
      "%s => %s",
      img.encoding.c_str(), output_encoding_.c_str());
    cvImg = cv_bridge::cvtColor(cvImg, output_encoding_);
  }
  if (parameters_.getVerticalFlip()) {
    cv::Mat flippedImg;
    cv::flip(cvImg->image, flippedImg, 0);
    cvImg->image = flippedImg;
  }
  auto down_image_size = parameters_.getDownsampling();
  if (!down_image_size.empty()) {
    cv::Mat resizedImg;
    auto width = down_image_size.at(0);
    auto height = down_image_size.at(1);
    cv::resize(cvImg->image, resizedImg, cv::Size(width, height), 0, 0, cv::INTER_AREA);
    cvImg->image = resizedImg;
  }

  // display vertical guide line in the center
  int center_x = cvImg->image.cols / 2;
  for (int i = 0; i < cvImg->image.rows; i += 10) {
      cv::line(cvImg->image, cv::Point(center_x, i), cv::Point(center_x, i + 5), cv::Scalar(0, 255, 0), 1);
  }

  auto outImg = std::make_unique<sensor_msgs::msg::Image>();
  cvImg->toImageMsg(*outImg);
  return outImg;
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

void V4L2Camera::capture_and_publish()
{
  RCLCPP_DEBUG(get_logger(), "Capture...");
  auto img = camera_->capture();
  if (img == nullptr) {
    // Failed capturing image, assume it is temporarily and continue a bit later
    return;
  }

  auto stamp = now();
  img = convert(*img);
  img->header.stamp = stamp;
  img->header.frame_id = camera_frame_id_;

  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
  if (!checkCameraInfo(*img, *ci)) {
    *ci = sensor_msgs::msg::CameraInfo{};
    ci->height = img->height;
    ci->width = img->width;
  }

  ci->header.stamp = stamp;

  if (get_node_options().use_intra_process_comms()) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Image message address [PUBLISH]:\t" << img.get());
    image_pub_->publish(std::move(img));
    info_pub_->publish(std::move(ci));
  } else {
    camera_transport_pub_.publish(*img, *ci);
  }
}

}  // namespace v4l2_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
