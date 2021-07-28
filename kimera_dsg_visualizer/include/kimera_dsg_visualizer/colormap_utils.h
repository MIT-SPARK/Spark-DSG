#pragma once
#include "kimera_dsg_visualizer/visualizer_types.h"

#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace kimera {
namespace dsg_utils {

inline std_msgs::ColorRGBA makeColorMsg(const NodeColor& color, double alpha = 1.0) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<double>(color(0)) / 255.0;
  msg.g = static_cast<double>(color(1)) / 255.0;
  msg.b = static_cast<double>(color(2)) / 255.0;
  msg.a = alpha;
  return msg;
}

inline double lerp(double min, double max, double ratio) {
  return (max - min) * ratio + min;
}

struct HlsColorMapConfig {
  double min_hue;
  double max_hue;
  double min_saturation;
  double max_saturation;
  double min_luminance;
  double max_luminance;
};

inline NodeColor interpolateColorMap(const HlsColorMapConfig& config, double ratio) {
  // override ratio input to be in [0, 1]
  ratio = std::clamp(ratio, 0.0, 1.0);

  cv::Mat hls_value(1, 1, CV_32FC3);
  // hue is in degrees, not [0, 1]
  hls_value.at<float>(0) = lerp(config.min_hue, config.max_hue, ratio) * 360.0;
  hls_value.at<float>(1) = lerp(config.min_luminance, config.max_luminance, ratio);
  hls_value.at<float>(2) = lerp(config.min_saturation, config.max_saturation, ratio);

  cv::Mat bgr;
  cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);

  NodeColor color;
  color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
  color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
  color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));
  return color;
}

}  // namespace dsg_utils
}  // namespace kimera
