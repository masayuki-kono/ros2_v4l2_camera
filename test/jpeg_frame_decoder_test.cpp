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

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

using v4l2_camera::JpegFrameDecoder;

namespace
{
constexpr int image_width = 64;
constexpr int image_height = 48;
// Quality of the JPEG frames the tests build; high, so that flat areas keep their level.
constexpr int jpeg_quality = 95;
// Level of the flat gray image.
constexpr int gray_level = 100;
// Highest level of a channel.
constexpr int max_level = 255;
// Seed of the random generator, fixed so that the noisy image is the same in every run.
constexpr uint64_t noise_seed = 12345;
// Luma (BT.601) of pure red: 0.299 * 255.
constexpr int luma_of_red = 76;
// Tolerance for the luma of a flat area after JPEG compression.
constexpr int luma_tolerance = 4;
// Tolerance for a channel of a flat area after JPEG compression and color conversion.
constexpr int color_tolerance = 8;
// Number of bytes in the frame body that the corrupt-body frame has between its markers.
constexpr std::size_t corrupt_body_size = 256;
// Sample positions away from the boundary between the left (red) and right (white) halves.
constexpr int red_x = 8;
constexpr int white_x = image_width - 8;
constexpr int sample_y = image_height / 2;
// Amount by which the size the caller expects exceeds the size of the frame.
constexpr int size_excess = 16;
// Size of the frame that is too short to hold the start and end markers (2 bytes each).
constexpr std::size_t short_frame_size = 1;
}  // namespace

class JpegFrameDecoderTest : public ::testing::Test
{
public:
  /// Defect of a frame that the decoder has to discard.
  enum class FrameDefect
  {
    ShortFrame,
    TruncatedFrame,
    MissingEndOfImage,
    MissingStartOfImage,
    NotAJpeg,
    CorruptBody,
    DifferentWidth,
    DifferentHeight,
  };

  struct RejectedFrameCase
  {
    std::string name;
    FrameDefect defect;
    /// Text that the reject reason of this defect contains; for a size defect, the text before the
    /// expected size.
    std::string reason_keyword;
  };

protected:
  /// Left half pure red, right half white, in BGR order.
  static cv::Mat redAndWhiteImage()
  {
    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar::all(max_level));
    image(cv::Rect(0, 0, image_width / 2, image_height)).setTo(cv::Scalar(0, 0, max_level));
    return image;
  }

  /// Random pixels (fixed seed), so that the JPEG is mostly image data.
  static cv::Mat noisyImage()
  {
    cv::Mat noise(image_height, image_width, CV_8UC3);
    cv::theRNG().state = noise_seed;
    cv::randu(noise, cv::Scalar::all(0), cv::Scalar::all(max_level));
    return noise;
  }

  static std::vector<uint8_t> encodeJpeg(const cv::Mat & image)
  {
    std::vector<uint8_t> jpeg;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    cv::imencode(".jpg", image, jpeg, params);
    return jpeg;
  }

  static sensor_msgs::msg::Image::UniquePtr decode(
    const std::vector<uint8_t> & frame, unsigned width, unsigned height,
    const std::string & output_encoding, std::string & reject_reason)
  {
    return JpegFrameDecoder::decode(
      frame.data(), frame.size(), width, height, output_encoding, reject_reason);
  }
};

/// output_encoding が mono8 のとき、カラーの JPEG から輝度だけを mono8 で展開することを確認する。
TEST_F(JpegFrameDecoderTest, decode_mono8_returns_luma_of_color_frame)
{
  // Arrange
  const auto frame = encodeJpeg(redAndWhiteImage());
  std::string reject_reason;

  // Act
  const auto image = decode(frame, image_width, image_height, "mono8", reject_reason);

  // Assert
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "mono8");
  EXPECT_EQ(image->width, static_cast<unsigned>(image_width));
  EXPECT_EQ(image->height, static_cast<unsigned>(image_height));
  EXPECT_EQ(image->step, static_cast<unsigned>(image_width));
  ASSERT_EQ(image->data.size(), static_cast<std::size_t>(image_width * image_height));
  EXPECT_NEAR(image->data[sample_y * image_width + red_x], luma_of_red, luma_tolerance);
  EXPECT_NEAR(image->data[sample_y * image_width + white_x], max_level, luma_tolerance);
}

/// グレースケールの JPEG でも mono8 で展開できることを確認する。
TEST_F(JpegFrameDecoderTest, decode_mono8_accepts_grayscale_frame)
{
  // Arrange
  const cv::Mat gray(image_height, image_width, CV_8UC1, cv::Scalar(gray_level));
  std::vector<uint8_t> frame;
  cv::imencode(".jpg", gray, frame);
  std::string reject_reason;

  // Act
  const auto image = decode(frame, image_width, image_height, "mono8", reject_reason);

  // Assert
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "mono8");
  EXPECT_NEAR(image->data[sample_y * image_width + red_x], gray_level, luma_tolerance);
}

class JpegFrameDecoderColorTest : public JpegFrameDecoderTest,
  public ::testing::WithParamInterface<std::string> {};

/// output_encoding がカラーのとき、カラーで展開して bgr8 で返すことを確認する。
TEST_P(JpegFrameDecoderColorTest, decode_color_encoding_returns_bgr8_frame)
{
  // Arrange
  const auto frame = encodeJpeg(redAndWhiteImage());
  std::string reject_reason;

  // Act
  const auto image = decode(frame, image_width, image_height, GetParam(), reject_reason);

  // Assert
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->encoding, "bgr8");
  EXPECT_EQ(image->step, static_cast<unsigned>(image_width * 3));
  ASSERT_EQ(image->data.size(), static_cast<std::size_t>(image_width * image_height * 3));
  const std::size_t red_pixel = (sample_y * image_width + red_x) * 3;
  EXPECT_NEAR(image->data[red_pixel + 0], 0, color_tolerance);          // B
  EXPECT_NEAR(image->data[red_pixel + 1], 0, color_tolerance);          // G
  EXPECT_NEAR(image->data[red_pixel + 2], max_level, color_tolerance);  // R
}

INSTANTIATE_TEST_SUITE_P(
  OutputEncodings, JpegFrameDecoderColorTest, ::testing::Values("bgr8", "rgb8"));

class JpegFrameDecoderRejectTest : public JpegFrameDecoderTest,
  public ::testing::WithParamInterface<JpegFrameDecoderTest::RejectedFrameCase>
{
protected:
  /// Builds the frame that has the given defect, starting from a valid frame.
  static std::vector<uint8_t> makeFrame(FrameDefect defect)
  {
    auto frame = encodeJpeg(redAndWhiteImage());
    switch (defect) {
      case FrameDefect::ShortFrame:
        // Not empty: the reason of an empty vector would depend on whether data() is null.
        frame.resize(short_frame_size);
        return frame;
      case FrameDefect::TruncatedFrame:
        // Cut inside the image data; a noisy image makes the image data most of the frame.
        frame = encodeJpeg(noisyImage());
        frame.resize(frame.size() / 2);
        return frame;
      case FrameDefect::MissingEndOfImage:
        // Only the end marker (2 bytes) is lost: libjpeg still decodes the rest.
        frame = encodeJpeg(noisyImage());
        frame.resize(frame.size() - 2);
        return frame;
      case FrameDefect::MissingStartOfImage:
        frame.erase(frame.begin());
        return frame;
      case FrameDefect::NotAJpeg:
        {
          std::vector<uint8_t> png;
          cv::imencode(".png", cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(0)), png);
          return png;
        }
      case FrameDefect::CorruptBody:
        // Both markers are present, but what lies between them is not a JPEG image.
        {
          std::vector<uint8_t> corrupt{0xFF, 0xD8};
          corrupt.insert(corrupt.end(), corrupt_body_size, 0x55);
          corrupt.insert(corrupt.end(), {0xFF, 0xD9});
          return corrupt;
        }
      default:
        // A valid frame; the defect is in the expected size.
        return frame;
    }
  }

  /// Width the caller expects; differs from the frame width for FrameDefect::DifferentWidth.
  static unsigned expectedWidth(FrameDefect defect)
  {
    return defect == FrameDefect::DifferentWidth ? image_width + size_excess : image_width;
  }

  /// Height the caller expects; differs from the frame height for FrameDefect::DifferentHeight.
  static unsigned expectedHeight(FrameDefect defect)
  {
    return defect == FrameDefect::DifferentHeight ? image_height + size_excess : image_height;
  }

  /// Text that the reject reason contains: the keyword, followed by the expected size for a size
  /// defect.
  static std::string expectedReason(const RejectedFrameCase & rejected)
  {
    const auto defect = rejected.defect;
    if (defect != FrameDefect::DifferentWidth && defect != FrameDefect::DifferentHeight) {
      return rejected.reason_keyword;
    }
    return rejected.reason_keyword + " " + std::to_string(expectedWidth(defect)) + "x" +
           std::to_string(expectedHeight(defect));
  }
};

/// 展開できないフレームを捨て（nullptr を返し）、捨てた検査を示す理由を返すことを確認する。
TEST_P(JpegFrameDecoderRejectTest, decode_rejected_frame_returns_nullptr_and_reason)
{
  // Arrange
  const auto defect = GetParam().defect;
  const auto frame = makeFrame(defect);
  const auto expected_reason = expectedReason(GetParam());
  std::string reject_reason;

  // Act
  const auto image = decode(
    frame, expectedWidth(defect), expectedHeight(defect), "mono8", reject_reason);

  // Assert
  EXPECT_EQ(image, nullptr);
  EXPECT_NE(reject_reason.find(expected_reason), std::string::npos) << reject_reason;
}

using Defect = JpegFrameDecoderTest::FrameDefect;
using RejectedFrame = JpegFrameDecoderTest::RejectedFrameCase;

INSTANTIATE_TEST_SUITE_P(
  RejectedFrames, JpegFrameDecoderRejectTest,
  ::testing::Values(
    RejectedFrame{"short_frame", Defect::ShortFrame, "frame shorter than the jpeg markers"},
    RejectedFrame{"truncated_frame", Defect::TruncatedFrame, "missing end-of-image"},
    RejectedFrame{"missing_end_of_image", Defect::MissingEndOfImage, "missing end-of-image"},
    RejectedFrame{"missing_start_of_image", Defect::MissingStartOfImage, "missing start-of-image"},
    RejectedFrame{"not_a_jpeg", Defect::NotAJpeg, "missing start-of-image"},
    RejectedFrame{"corrupt_body", Defect::CorruptBody, "decoding produced no image"},
    RejectedFrame{"different_width", Defect::DifferentWidth, "differs from expected"},
    RejectedFrame{"different_height", Defect::DifferentHeight, "differs from expected"}),
  [](const ::testing::TestParamInfo<RejectedFrame> & info) {return info.param.name;});
