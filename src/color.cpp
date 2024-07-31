#include "spark_dsg/color.h"

#include <algorithm>
#include <random>

namespace spark_dsg {

namespace colormaps {

static const std::vector<Color> ironbow = {
    {0, 0, 0},
    {145, 20, 145},
    {255, 138, 0},
    {255, 230, 40},
    {255, 255, 255},
};

static const std::vector<Color> color_brewer_paired{
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

}  // namespace colormaps

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<uint8_t> dis(0, 255);

namespace {

struct HueInfo {
  float max = 0.0f;
  float min = 1.0f;
  float hue = 0.0;
};

std::array<float, 3> rgbFromChromaHue(float chroma, float hue) {
  const float hue_prime = hue * 6.0f;
  const float x = chroma * (1.0f - std::abs(std::fmod(hue_prime, 2) - 1.0f));
  if (hue_prime < 1) {
    return {chroma, x, 0};
  } else if (hue_prime < 2) {
    return {x, chroma, 0};
  } else if (hue_prime < 3) {
    return {0, chroma, x};
  } else if (hue_prime < 4) {
    return {0, x, chroma};
  } else if (hue_prime < 5) {
    return {x, 0, chroma};
  }
  return {chroma, 0, x};
}

// see https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html for details
HueInfo getHue(const Color& color) {
  // TODO(nathan) think about tie-breaking
  std::array<float, 3> rgb{color.r / 255.0f, color.g / 255.0f, color.b / 255.0f};

  HueInfo info;
  size_t max_idx = 0;
  for (size_t c = 0; c < 3; ++c) {
    if (rgb[c] > info.max) {
      info.max = rgb[c];
      max_idx = c;
    }

    if (rgb[c] < info.min) {
      info.min = rgb[c];
    }
  }

  if (color.r == color.g && color.g == color.b) {
    // maps to central axis, but we define it as 0.0
    info.hue = 0.0;
  } else if (max_idx == 0 && color.r == color.g) {
    // should be equidistant between red and green
    info.hue = 1.0f / 6.0f;
  } else if (max_idx == 0 && color.r == color.b) {
    // should be equidistant between blue and red
    info.hue = 5.0f / 6.0f;
  } else if (max_idx == 1 && color.g == color.b) {
    // should be equidistant between green and blue
    info.hue = 0.5f;
  } else {
    const auto l_idx = (max_idx + 1) % 3;
    const auto r_idx = (max_idx + 2) % 3;
    const auto chroma = info.max - info.min;
    info.hue = (rgb[l_idx] - rgb[r_idx]) / (2.0f * chroma) + max_idx;
    info.hue /= 3.0;
  }

  if (info.hue < 0.0f) {
    info.hue += 1.0f;
  }

  return info;
}

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

bool Color::operator==(const Color& other) const {
  return r == other.r && g == other.g && b == other.b && a == other.a;
}

bool Color::operator<(const Color& other) const {
  if (r != other.r) {
    return r < other.r;
  }
  if (g != other.g) {
    return g < other.g;
  }
  if (b != other.b) {
    return b < other.b;
  }
  return a < other.a;
}

void Color::merge(const Color& other, float weight) { *this = blend(other, weight); }

Color Color::blend(const Color& other, float weight) const {
  weight = std::clamp(weight, 0.0f, 1.0f);
  return Color(static_cast<uint8_t>(r * (1.0f - weight) + other.r * weight),
               static_cast<uint8_t>(g * (1.0f - weight) + other.g * weight),
               static_cast<uint8_t>(b * (1.0f - weight) + other.b * weight),
               static_cast<uint8_t>(a * (1.0f - weight) + other.a * weight));
}

Color Color::random() { return Color(dis(gen), dis(gen), dis(gen), dis(gen)); }

std::array<float, 4> Color::toUnitRange() const {
  return {r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f};
}

Color Color::fromHSV(float hue, float saturation, float value) {
  hue = std::clamp(hue, 0.0f, 1.0f);
  saturation = std::clamp(saturation, 0.0f, 1.0f);
  value = std::clamp(value, 0.0f, 1.0f);
  const float chroma = value * saturation;
  const auto [r1, g1, b1] = rgbFromChromaHue(chroma, hue);
  const float m = value - chroma;
  return Color(fromUnitRange(r1 + m), fromUnitRange(g1 + m), fromUnitRange(b1 + m));
}

// see https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html for details
std::array<float, 3> Color::toHSV() const {
  const auto info = getHue(*this);

  std::array<float, 3> hsv;
  hsv[0] = info.hue;
  hsv[1] = info.max == 0.0f ? 0.0f : static_cast<float>(info.max - info.min) / info.max;
  hsv[2] = info.max;
  return hsv;
}

Color Color::fromHLS(float hue, float luminance, float saturation) {
  hue = std::clamp(hue, 0.0f, 1.0f);
  luminance = std::clamp(luminance, 0.0f, 1.0f);
  saturation = std::clamp(saturation, 0.0f, 1.0f);
  const float chroma = (1.0f - std::abs(2.0f * luminance - 1.0f)) * saturation;
  const auto [r1, g1, b1] = rgbFromChromaHue(chroma, hue);
  const float m = luminance - chroma / 2.0f;
  return Color(fromUnitRange(r1 + m), fromUnitRange(g1 + m), fromUnitRange(b1 + m));
}

// see https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html for details
std::array<float, 3> Color::toHLS() const {
  const auto info = getHue(*this);

  std::array<float, 3> hls;
  hls[0] = info.hue;
  hls[1] = (info.max + info.min) / 2.0f;
  if (hls[1] == 0.0f || hls[1] == 1.0f) {
    hls[2] = 0.0f;
  } else if (hls[1] < 0.5f) {
    hls[2] = (info.max - info.min) / (info.max + info.min);
  } else {
    hls[2] = (info.max - info.min) / (2.0f - info.max - info.min);
  }

  return hls;
}

Color Color::gray(float value) {
  const auto char_value = fromUnitRange(std::clamp(value, 0.0f, 1.0f));
  return Color(char_value, char_value, char_value);
}

Color Color::quality(float value) {
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

Color Color::spectrum(float value, const std::vector<Color>& colors) {
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

Color Color::ironbow(float value) { return spectrum(value, colormaps::ironbow); }

Color Color::rainbow(float value) {
  return fromHLS(std::clamp(value, 0.0f, 1.0f), 0.5f, 1.0f);
}

Color Color::rainbowId(size_t id, size_t ids_per_revolution) {
  return Color::rainbow(exponentialOffsetId(id, ids_per_revolution));
}

Color Color::colorbrewer(size_t id) {
  const auto& cmap = colormaps::color_brewer_paired;
  return cmap.at(id % cmap.size());
}

std::ostream& operator<<(std::ostream& out, const Color& color) {
  out << "[RGBA: " << static_cast<int>(color.r) << ", " << static_cast<int>(color.g)
      << ", " << static_cast<int>(color.b) << ", " << static_cast<int>(color.a) << "]";
  return out;
}

}  // namespace spark_dsg
