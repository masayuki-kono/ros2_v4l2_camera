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

#ifndef V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/image.hpp>

#include "v4l2_camera/control.hpp"
#include "v4l2_camera/image_format.hpp"
#include "v4l2_camera/pixel_format.hpp"

namespace v4l2_camera
{

/** Camera device using Video4Linux2
 */
class V4l2CameraDevice
{
public:
  explicit V4l2CameraDevice(std::string device, double capture_timeout_sec);

  bool open();
  bool close();
  bool start();
  bool stop();

  // Query properties and current state of a control
  Control queryControl(uint32_t id, bool silent = false);

  // Get all pre-queried controls
  auto const & getControls() const {return controls_;}

  // Get current control value
  int32_t getControlValue(uint32_t id) const;

  // Attempt to set current control value
  bool setControlValue(uint32_t id, int32_t value);

  // Types used to describe available image sizes
  enum class ImageSizeType
  {
    DISCRETE,
    STEPWISE,
    CONTINUOUS
  };
  using ImageSizesVector = std::vector<std::pair<uint16_t, uint16_t>>;
  using ImageSizesDescription = std::pair<ImageSizeType, ImageSizesVector>;

  auto const & getImageFormats() const {return image_formats_;}
  auto const & getImageSizes() const {return image_sizes_;}
  auto const & getCurrentDataFormat() const {return cur_data_format_;}
  bool requestDataFormat(PixelFormat const & format);

  std::string getCameraName();

  /**
   * @brief Captures one frame.
   *
   * Frames of a fixed-size pixel format are returned as they come from the camera. MJPG frames,
   * whose length changes from frame to frame, are decoded with JpegFrameDecoder.
   *
   * @param output_encoding Requested output encoding; selects how an MJPG frame is decoded.
   * @return The frame, or nullptr when no valid frame was captured (timeout, device error,
   *   invalid or undecodable frame).
   */
  sensor_msgs::msg::Image::UniquePtr capture(std::string const & output_encoding);

  /**
   * @brief Checks one dequeued frame and converts it to an image.
   *
   * A fixed-size frame must hold at least bytesPerLine * height bytes and is copied as it is;
   * only the first bytesPerLine * height bytes are used. An MJPG frame has no fixed length and
   * is decoded with JpegFrameDecoder instead. A frame of an unsupported pixel format is returned
   * with an empty encoding.
   *
   * @param data First byte of the frame. Not retained after the call.
   * @param bytes_used Number of bytes the driver wrote to the buffer.
   * @param buffer_length Size of the buffer [byte].
   * @param buffer_flags Flags the driver set on the dequeued buffer (V4L2_BUF_FLAG_*).
   * @param format Format the frame was captured in.
   * @param output_encoding Requested output encoding; selects how an MJPG frame is decoded.
   * @param reject_reason Set to the log message that tells why the frame was discarded when
   *   nullptr is returned; left unchanged otherwise.
   * @return The image, or nullptr when the frame is discarded.
   */
  static sensor_msgs::msg::Image::UniquePtr convertFrame(
    const uint8_t * data, std::size_t bytes_used, std::size_t buffer_length,
    uint32_t buffer_flags, PixelFormat const & format, std::string const & output_encoding,
    std::string & reject_reason);

private:
  /// Image buffer
  struct Buffer
  {
    unsigned index;
    unsigned char * start;
    size_t length;
  };

  std::string device_;
  int fd_;
  // Timeout for capture operation in seconds
  const double capture_timeout_sec_;

  v4l2_capability capabilities_;
  std::vector<ImageFormat> image_formats_;
  std::map<unsigned, ImageSizesDescription> image_sizes_;
  std::vector<Control> controls_;

  PixelFormat cur_data_format_;

  std::vector<Buffer> buffers_;

  // Requests and stores all formats available for this camera
  void listImageFormats();

  // Requests and stores all frame sizes available for this camera
  void listImageSizes();

  ImageSizesDescription listDiscreteImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listStepwiseImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listContinuousImageSizes(v4l2_frmsizeenum frm_size_enum);

  // Requests and stores all controls available for this camera
  void listControls();

  // Set up memory mapping to buffers
  bool initMemoryMapping();

  /**
   * @brief Checks one dequeued MJPG frame and decodes it.
   *
   * @param data First byte of the frame.
   * @param bytes_used Number of bytes the driver wrote to the buffer.
   * @param buffer_length Size of the buffer [byte].
   * @param buffer_flags Flags the driver set on the dequeued buffer (V4L2_BUF_FLAG_*).
   * @param format Format the frame was captured in.
   * @param output_encoding Requested output encoding.
   * @param reject_reason Set to the log message that tells why the frame was discarded when
   *   nullptr is returned; left unchanged otherwise.
   * @return The decoded image, or nullptr when the frame is discarded.
   */
  static sensor_msgs::msg::Image::UniquePtr decodeMjpgFrame(
    const uint8_t * data, std::size_t bytes_used, std::size_t buffer_length,
    uint32_t buffer_flags, PixelFormat const & format, std::string const & output_encoding,
    std::string & reject_reason);
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
