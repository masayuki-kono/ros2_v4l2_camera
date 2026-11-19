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

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <gtest/gtest.h>
#include <linux/videodev2.h>

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

using v4l2_camera::PixelFormat;
using v4l2_camera::V4l2CameraDevice;

namespace
{
constexpr unsigned image_width = 64;
constexpr unsigned image_height = 48;
// Quality of the JPEG frames the tests build.
constexpr int jpeg_quality = 95;
// Level of the flat images; the noisy image has a different, larger JPEG.
constexpr int flat_level = 200;
// Tolerance for the level of a flat area after JPEG compression.
constexpr int level_tolerance = 4;
// Bytes per line that a driver could report for MJPG; the frames below are shorter than
// bytesPerLine * image_height, so a fixed-size check would discard them.
constexpr unsigned long_bytes_per_line = 1024;
// Bytes that the buffer holds beyond the largest frame.
constexpr std::size_t spare_buffer_bytes = 4096;
// Position of a sample pixel.
constexpr unsigned sample_index = (image_height / 2) * image_width + image_width / 2;
// Level of the one pixel that stands out from the all-zero GRAY frame, so that a conversion that
// loses or moves it is noticed.
constexpr uint8_t marker_level = 77;
// Amount by which the width the caller expects exceeds the width of the frame.
constexpr unsigned width_excess = 16;
// Highest level of a channel.
constexpr int max_level = 255;
// Seed of the random generator, fixed so that the noisy image is the same in every run.
constexpr uint64_t noise_seed = 12345;
}  // namespace

class V4l2CameraDeviceTest : public ::testing::Test
{
protected:
  static PixelFormat makeFormat(uint32_t pixel_format, unsigned bytes_per_line)
  {
    v4l2_pix_format raw{};
    raw.width = image_width;
    raw.height = image_height;
    raw.pixelformat = pixel_format;
    raw.bytesperline = bytes_per_line;
    return PixelFormat{raw};
  }

  static std::vector<uint8_t> flatJpeg()
  {
    return encodeJpeg(cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar::all(flat_level)));
  }

  /// Random pixels (fixed seed), whose JPEG is much longer than the flat one.
  static std::vector<uint8_t> noisyJpeg()
  {
    cv::Mat noise(image_height, image_width, CV_8UC3);
    cv::theRNG().state = noise_seed;
    cv::randu(noise, cv::Scalar::all(0), cv::Scalar::all(max_level));
    return encodeJpeg(noise);
  }

  /// convertFrame() of a frame whose buffer has the given length and flags.
  static sensor_msgs::msg::Image::UniquePtr convert(
    const std::vector<uint8_t> & frame, std::size_t buffer_length, uint32_t buffer_flags,
    const PixelFormat & format, const std::string & output_encoding, std::string & reject_reason)
  {
    return V4l2CameraDevice::convertFrame(
      frame.data(), frame.size(), buffer_length, buffer_flags, format, output_encoding,
      reject_reason);
  }

private:
  static std::vector<uint8_t> encodeJpeg(const cv::Mat & image)
  {
    std::vector<uint8_t> jpeg;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    cv::imencode(".jpg", image, jpeg, params);
    return jpeg;
  }
};

/// GRAY のフレームが期待する長さのとき、mono8 の画像にそのまま変換することを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_gray_frame_of_expected_size_returns_mono8_image)
{
  // Arrange
  std::vector<uint8_t> frame(image_width * image_height, 0);
  frame[sample_index] = marker_level;
  const auto format = makeFormat(V4L2_PIX_FMT_GREY, image_width);
  std::string reject_reason;

  // Act
  const auto image = convert(frame, frame.size(), 0, format, "mono8", reject_reason);

  // Assert
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "mono8");
  EXPECT_EQ(image->width, image_width);
  EXPECT_EQ(image->height, image_height);
  EXPECT_EQ(image->step, image_width);
  EXPECT_EQ(image->data, frame);
}

/// GRAY のフレームが期待する長さに満たないとき、捨てて理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_gray_frame_shorter_than_expected_is_discarded)
{
  // Arrange
  const std::vector<uint8_t> frame(image_width * image_height - 1, 0);
  const auto format = makeFormat(V4L2_PIX_FMT_GREY, image_width);
  std::string reject_reason;

  // Act
  const auto image = convert(frame, frame.size(), 0, format, "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find("Discarding invalid frame"), std::string::npos) << reject_reason;
}

/// GRAY のフレームの使用バイト数がバッファの長さを超えるとき、捨てて理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_gray_frame_longer_than_buffer_is_discarded)
{
  // Arrange
  const std::vector<uint8_t> frame(image_width * image_height, 0);
  const auto format = makeFormat(V4L2_PIX_FMT_GREY, image_width);
  std::string reject_reason;

  // Act
  const auto image = convert(frame, frame.size() - 1, 0, format, "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find("Discarding invalid frame"), std::string::npos) << reject_reason;
}

class V4l2CameraDeviceMjpgTest : public V4l2CameraDeviceTest,
  public ::testing::WithParamInterface<unsigned> {};

/// MJPG の長さの違うフレームを、bytesPerLine × height に満たなくても展開することを確認する。
TEST_P(V4l2CameraDeviceMjpgTest, convertFrame_mjpg_frames_of_different_lengths_are_decoded)
{
  // Arrange
  const auto flat_frame = flatJpeg();
  const auto noisy_frame = noisyJpeg();
  const auto format = makeFormat(V4L2_PIX_FMT_MJPEG, GetParam());
  const std::size_t buffer_length = noisy_frame.size() + spare_buffer_bytes;
  ASSERT_NE(flat_frame.size(), noisy_frame.size());
  ASSERT_LT(noisy_frame.size(), static_cast<std::size_t>(long_bytes_per_line) * image_height);
  std::string reject_reason;

  // Act
  const auto flat_image = convert(flat_frame, buffer_length, 0, format, "mono8", reject_reason);
  const auto noisy_image = convert(noisy_frame, buffer_length, 0, format, "mono8", reject_reason);

  // Assert
  ASSERT_NE(flat_image, nullptr) << reject_reason;
  ASSERT_NE(noisy_image, nullptr) << reject_reason;
  EXPECT_EQ(flat_image->encoding, "mono8");
  EXPECT_EQ(flat_image->data.size(), static_cast<std::size_t>(image_width * image_height));
  EXPECT_EQ(noisy_image->data.size(), static_cast<std::size_t>(image_width * image_height));
  EXPECT_NEAR(flat_image->data[sample_index], flat_level, level_tolerance);
}

// 0 は uvcvideo が MJPG で報告する bytesPerLine。もう一方は固定長の検査に掛かる長さ。
INSTANTIATE_TEST_SUITE_P(
  BytesPerLine, V4l2CameraDeviceMjpgTest, ::testing::Values(0U, long_bytes_per_line));

/// MJPG のエラーフラグ付きのフレームを、展開できる内容でも捨てて理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_mjpg_frame_flagged_as_error_is_discarded)
{
  // Arrange
  const auto frame = flatJpeg();
  const auto format = makeFormat(V4L2_PIX_FMT_MJPEG, 0);
  std::string reject_reason;

  // Act
  const auto image = convert(
    frame, frame.size() + spare_buffer_bytes, V4L2_BUF_FLAG_ERROR, format, "mono8",
    reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find("driver flagged the frame as erroneous"), std::string::npos) <<
    reject_reason;
}

/// MJPG の使用バイト数が 0 のフレームを捨てて理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_mjpg_frame_without_bytes_is_discarded)
{
  // Arrange
  const auto frame = flatJpeg();
  const auto format = makeFormat(V4L2_PIX_FMT_MJPEG, 0);
  std::string reject_reason;

  // Act
  const auto image = V4l2CameraDevice::convertFrame(
    frame.data(), 0, frame.size() + spare_buffer_bytes, 0, format, "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find("frame is empty"), std::string::npos) << reject_reason;
}

/// MJPG の使用バイト数がバッファの長さを超えるフレームを捨てて理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_mjpg_frame_longer_than_buffer_is_discarded)
{
  // Arrange
  const auto frame = flatJpeg();
  const auto format = makeFormat(V4L2_PIX_FMT_MJPEG, 0);
  std::string reject_reason;

  // Act
  const auto image = convert(frame, frame.size() - 1, 0, format, "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find("bytesused exceeds the buffer length"), std::string::npos) <<
    reject_reason;
}

/// MJPG の展開できないフレームを捨て、展開後の大きさを含む理由を返すことを確認する。
TEST_F(V4l2CameraDeviceTest, convertFrame_mjpg_frame_of_unexpected_size_is_discarded)
{
  // Arrange
  const auto frame = flatJpeg();
  v4l2_pix_format raw{};
  raw.width = image_width + width_excess;
  raw.height = image_height;
  raw.pixelformat = V4L2_PIX_FMT_MJPEG;
  const std::string expected_reason =
    "decoded size " + std::to_string(image_width) + "x" + std::to_string(image_height) +
    " differs from expected " + std::to_string(image_width + width_excess) + "x" +
    std::to_string(image_height);
  std::string reject_reason;

  // Act
  const auto image = convert(
    frame, frame.size() + spare_buffer_bytes, 0, PixelFormat{raw}, "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find(expected_reason), std::string::npos) << reject_reason;
}
