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

#include "v4l2_camera/jpeg_frame_decoder.hpp"

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

using v4l2_camera::JpegFrameDecoder;
using sensor_msgs::msg::Image;

namespace
{
/// First byte of every JPEG marker.
constexpr uint8_t marker_prefix = 0xFF;

/// Second byte of the marker that starts a JPEG image.
constexpr uint8_t start_of_image = 0xD8;

/// Second byte of the marker that ends a JPEG image.
constexpr uint8_t end_of_image = 0xD9;
}  // namespace

Image::UniquePtr JpegFrameDecoder::decode(
  const uint8_t * data, std::size_t size, unsigned width, unsigned height,
  const std::string & output_encoding, std::string & reject_reason)
{
  if (data == nullptr) {
    reject_reason = "no frame data";
    return nullptr;
  }
  if (!hasStartAndEndMarkers(data, size, reject_reason)) {
    return nullptr;
  }

  // Decoding only the luminance is much cheaper than decoding in color and converting to gray.
  const bool decode_gray = output_encoding == sensor_msgs::image_encodings::MONO8;
  const int flags =
    (decode_gray ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR) | cv::IMREAD_IGNORE_ORIENTATION;

  // The cv::Mat only wraps the frame; imdecode() does not modify or retain it.
  const cv::Mat encoded(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t *>(data));
  cv::Mat decoded;
  try {
    decoded = cv::imdecode(encoded, flags);
  } catch (const cv::Exception & e) {
    reject_reason = std::string("opencv exception(") + e.what() + ")";
    return nullptr;
  }

  if (decoded.empty()) {
    reject_reason = "decoding produced no image";
    return nullptr;
  }
  if (decoded.cols != static_cast<int>(width) || decoded.rows != static_cast<int>(height)) {
    reject_reason = "decoded size " + std::to_string(decoded.cols) + "x" +
      std::to_string(decoded.rows) + " differs from expected " + std::to_string(width) + "x" +
      std::to_string(height);
    return nullptr;
  }

  auto image = std::make_unique<Image>();
  image->width = width;
  image->height = height;
  image->encoding =
    decode_gray ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
  image->is_bigendian = false;
  image->step = static_cast<uint32_t>(decoded.cols * decoded.channels());
  image->data.assign(decoded.data, decoded.data + decoded.total() * decoded.elemSize());
  return image;
}

bool JpegFrameDecoder::hasStartAndEndMarkers(
  const uint8_t * data, std::size_t size, std::string & reject_reason)
{
  // libjpeg fills the missing part of a cut-short frame and still reports success, so the end
  // marker is the only way to notice that a frame was cut short.
  constexpr std::size_t marker_size = 2;
  if (size < marker_size * 2) {
    reject_reason = "frame shorter than the jpeg markers(size=" + std::to_string(size) + ")";
    return false;
  }
  if (data[0] != marker_prefix || data[1] != start_of_image) {
    reject_reason = "missing start-of-image marker";
    return false;
  }
  if (data[size - 2] != marker_prefix || data[size - 1] != end_of_image) {
    reject_reason = "missing end-of-image marker";
    return false;
  }
  return true;
}
