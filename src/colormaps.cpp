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

static const std::vector<Color> custom_150_palette{
    {0, 102, 51},    {0, 255, 0},     {4, 74, 246},    {191, 191, 181}, {72, 90, 13},
    {204, 0, 102},   {196, 26, 252},  {254, 217, 251}, {127, 255, 255}, {133, 87, 54},
    {130, 26, 50},   {10, 5, 65},     {3, 75, 135},    {18, 124, 89},   {21, 182, 126},
    {90, 171, 130},  {189, 217, 119}, {188, 63, 11},   {86, 82, 249},   {196, 221, 253},
    {187, 78, 62},   {188, 49, 167},  {182, 161, 248}, {112, 201, 196}, {11, 41, 99},
    {211, 102, 1},   {4, 227, 101},   {147, 201, 250}, {255, 0, 0},     {224, 33, 3},
    {254, 74, 17},   {255, 127, 0},   {176, 188, 66},  {247, 29, 54},   {74, 117, 80},
    {71, 56, 199},   {250, 107, 216}, {4, 204, 229},   {25, 208, 53},   {52, 249, 65},
    {70, 220, 121},  {68, 86, 156},   {7, 150, 36},    {255, 127, 127}, {64, 46, 6},
    {139, 167, 123}, {18, 165, 78},   {157, 166, 28},  {9, 163, 248},   {191, 67, 218},
    {135, 243, 61},  {81, 119, 190},  {0, 126, 204},   {228, 73, 252},  {227, 108, 163},
    {127, 255, 127}, {50, 211, 203},  {3, 190, 177},   {127, 255, 0},   {11, 59, 10},
    {108, 38, 143},  {243, 33, 254},  {182, 106, 251}, {120, 251, 191}, {145, 115, 208},
    {48, 129, 250},  {43, 78, 65},    {73, 0, 197},    {0, 249, 143},   {80, 160, 69},
    {200, 252, 62},  {127, 39, 198},  {139, 79, 186},  {155, 225, 169}, {239, 148, 63},
    {30, 183, 5},    {137, 90, 3},    {49, 35, 155},   {242, 186, 113}, {127, 0, 0},
    {191, 148, 1},   {134, 147, 65},  {175, 254, 104}, {84, 183, 23},   {12, 87, 183},
    {1, 20, 178},    {185, 224, 0},   {255, 255, 127}, {44, 252, 176},  {62, 186, 252},
    {3, 253, 46},    {41, 208, 156},  {138, 94, 126},  {32, 31, 231},   {192, 120, 193},
    {231, 66, 87},   {105, 0, 153},   {251, 216, 190}, {228, 45, 198},  {0, 0, 255},
    {67, 234, 247},  {156, 4, 152},   {253, 219, 57},  {183, 148, 126}, {246, 154, 179},
    {127, 127, 255}, {0, 255, 255},   {225, 189, 224}, {47, 253, 125},  {47, 0, 127},
    {92, 207, 58},   {173, 253, 235}, {251, 78, 170},  {164, 41, 91},   {0, 127, 0},
    {65, 43, 54},    {196, 100, 129}, {143, 157, 184}, {234, 190, 2},   {0, 234, 191},
    {191, 254, 179}, {149, 1, 215},   {67, 231, 11},   {131, 195, 0},   {188, 121, 62},
    {70, 143, 13},   {106, 168, 230}, {231, 106, 88},  {255, 255, 0},   {121, 54, 15},
    {127, 255, 0},   {248, 157, 252}, {63, 7, 14},     {146, 63, 249},  {185, 8, 32},
    {247, 0, 223},   {234, 252, 199}, {91, 29, 248},   {122, 0, 89},    {41, 99, 220},
    {28, 132, 140},  {127, 209, 111}, {110, 54, 79},   {63, 68, 107},   {214, 0, 171},
    {71, 9, 82},     {224, 187, 57},  {253, 20, 130},  {127, 127, 0},   {113, 132, 132},
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

Color ironbow(float value) { return spectrum(value, ironbow_palette); }

Color rainbow(float value) {
  return Color::fromHLS(std::clamp(value, 0.0f, 1.0f), 0.5f, 1.0f);
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

Color divergent(float value,
                float hue_low,
                float hue_high,
                float saturation,
                float luminance,
                bool dark) {
  value = std::clamp(value, 0.0f, 1.0f);
  const float alpha = value > 0.5f ? 2.0f * (value - 0.5f) : 2.0f * (0.5f - value);
  return Color::fromHLS(value > 0.5f ? hue_high : hue_low,
                        dark ? alpha * luminance : (alpha * luminance) + 1.0f - alpha,
                        alpha * saturation);
}

Color rainbowId(size_t id, size_t ids_per_revolution) {
  return rainbow(exponentialOffsetId(id, ids_per_revolution));
}

Color colorbrewerId(size_t id) {
  const auto& cmap = colorbrewer_palette;
  return cmap.at(id % cmap.size());
}

Color distinct150Id(size_t id) {
  const auto& cmap = custom_150_palette;
  return cmap.at(id % cmap.size());
}

const std::vector<Color>& colorbrewerPalette() { return colorbrewer_palette; }

const std::vector<Color>& distinct150Palette() { return custom_150_palette; }

}  // namespace spark_dsg::colormaps
