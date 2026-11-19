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

#ifndef V4L2_CAMERA__JPEG_FRAME_DECODER_HPP_
#define V4L2_CAMERA__JPEG_FRAME_DECODER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <sensor_msgs/msg/image.hpp>

namespace v4l2_camera
{

/** Decoder for MJPG (Motion-JPEG) frames
 *
 * One MJPG frame is one complete JPEG image whose length changes from frame to frame.
 */
class JpegFrameDecoder
{
public:
  /**
   * @brief Decodes one MJPG frame into an uncompressed image.
   *
   * When output_encoding is "mono8", only the luminance is decoded and the image is returned as
   * "mono8". For any other output_encoding the image is decoded in color and returned as "bgr8";
   * the caller converts it to output_encoding like it does for the other pixel formats.
   *
   * A frame is rejected when it does not start with the JPEG start-of-image marker, does not end
   * with the end-of-image marker (e.g. it was cut short by a transfer error), cannot be decoded,
   * or does not have the expected size.
   *
   * @param data First byte of the frame. Not retained after the call.
   * @param size Number of bytes in the frame; may differ from frame to frame.
   * @param width Expected image width [pixel].
   * @param height Expected image height [pixel].
   * @param output_encoding Requested ROS image encoding of the output.
   * @param reject_reason Set to why the frame was rejected, with the decoder's own message and
   *   the decoded size where it applies, when nullptr is returned; left unchanged otherwise.
   * @return The decoded image, or nullptr when the frame is rejected.
   */
  static sensor_msgs::msg::Image::UniquePtr decode(
    const uint8_t * data, std::size_t size, unsigned width, unsigned height,
    const std::string & output_encoding, std::string & reject_reason);

private:
  /**
   * @brief Checks that a frame starts and ends with the JPEG start and end markers.
   *
   * @param data First byte of the frame.
   * @param size Number of bytes in the frame.
   * @param reject_reason Set to the missing marker when false is returned; left unchanged
   *   otherwise.
   * @return true if both markers are present.
   */
  static bool hasStartAndEndMarkers(
    const uint8_t * data, std::size_t size, std::string & reject_reason);
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__JPEG_FRAME_DECODER_HPP_
