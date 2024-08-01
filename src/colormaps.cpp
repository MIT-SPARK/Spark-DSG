#include "spark_dsg/colormaps.h"

#include <algorithm>
#include <random>

namespace spark_dsg::colormaps {

namespace {

/**
 * @brief Map a potentially infinite number of ids to a never repeating pattern in [0,
 * 1].
 */
float exponentialOffsetId(size_t id, size_t ids_per_revolution) {
  const size_t revolution = id / ids_per_revolution;
  const float progress_along_revolution =
      std::fmod(static_cast<float>(id) / ids_per_revolution, 1.f);
  float offset = 0.0f;
  if (ids_per_revolution < id + 1u) {
    const size_t current_episode = std::floor(std::log2(revolution));
    const size_t episode_start = std::exp2(current_episode);
    const size_t current_subdivision = revolution - episode_start;
    const float subdivision_step_size = 1.0f / (ids_per_revolution * 2 * episode_start);
    offset = (2.0f * current_subdivision + 1) * subdivision_step_size;
  }
  return progress_along_revolution + offset;
}

// remap unit-range color value to nearest uint8_t value
inline uint8_t fromUnitRange(float value) {
  return static_cast<uint8_t>(std::clamp(std::round(255.0f * value), 0.0f, 255.0f));
}

}  // namespace

static const std::vector<Color> ironbow_palette = {
    {0, 0, 0},
    {145, 20, 145},
    {255, 138, 0},
    {255, 230, 40},
    {255, 255, 255},
};

static const std::vector<Color> colorbrewer_palette{
    {166, 206, 227},
    {31, 120, 180},
    {178, 223, 138},
    {51, 160, 44},
    {251, 154, 153},
    {227, 26, 28},
    {253, 191, 111},
    {255, 127, 0},
    {202, 178, 214},
    {106, 61, 154},
    {255, 255, 153},
    {177, 89, 40},
};

Color gray(float value) {
  const auto char_value = fromUnitRange(std::clamp(value, 0.0f, 1.0f));
  return Color(char_value, char_value, char_value);
}

Color quality(float value) {
  value = std::clamp(value, 0.0f, 1.0f);
  Color color;
  if (value > 0.5f) {
    color.r = fromUnitRange((1.0f - value) * 2.0f);
    color.g = 255;
  } else {
    color.r = 255;
    color.g = fromUnitRange(value * 2.0f);
  }

  return color;
}

Color spectrum(float value, const std::vector<Color>& colors) {
  if (colors.empty()) {
    return Color::black();
  }

  value = std::clamp(value, 0.0f, 1.0f);
  const size_t num_steps = colors.size() - 1;
  const size_t index = static_cast<size_t>(value * num_steps);
  if (index >= num_steps) {
    return colors.at(num_steps);
  }

  const float weight = value * num_steps - index;
  return colors.at(index).blend(colors.at(index + 1), weight);
}

Color hls(float value, const Color& start, const Color& end) {
  const auto start_hls = start.toHLS();
  const auto end_hls = end.toHLS();
  return hls(value, start_hls, end_hls);
}

Color hls(float value,
          const std::array<float, 3>& start,
          const std::array<float, 3>& end) {
  const auto& [h1, l1, s1] = start;
  const auto& [h2, l2, s2] = end;
  const auto w = std::clamp(value, 0.0f, 1.0f);
  const auto w_inv = 1.0f - w;
  // interpolation is convex combination of hls values
  return Color::fromHLS(w * h1 + w_inv * h2, w * l1 + w_inv * l2, w * s1 + w_inv * s2);
}

Color ironbow(float value) { return spectrum(value, ironbow_palette); }

Color rainbow(float value) {
  return Color::fromHLS(std::clamp(value, 0.0f, 1.0f), 0.5f, 1.0f);
}

Color rainbowId(size_t id, size_t ids_per_revolution) {
  return rainbow(exponentialOffsetId(id, ids_per_revolution));
}

Color colorbrewer(size_t id) {
  const auto& cmap = colorbrewer_palette;
  return cmap.at(id % cmap.size());
}

}  // namespace spark_dsg::colormaps
