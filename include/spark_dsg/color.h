/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once

#include <cstdint>
#include <ostream>
#include <vector>

namespace spark_dsg {

struct Color {
  // Data.
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t a = 255;

  // Cosntructors.
  Color() = default;
  Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) : r(r), g(g), b(b), a(a) {}
  virtual ~Color() = default;

  // Operators.
  bool operator==(const Color& other) const;
  bool operator!=(const Color& other) const { return !(*this == other); }
  bool operator<(const Color& other) const;
  bool operator>(const Color& other) const { return other < *this; }
  friend std::ostream& operator<<(std::ostream&, const Color&);

  // Tools.
  /**
   * @brief Blend another color with this one.
   * @param other The other color to blend.
   * @param weight The weight of the other color [0,1].
   * @return The resulting blended color.
   */
  Color blend(const Color& other, float weight = 0.5f) const;

  /**
   * @brief Merge another color into this one.
   * @param other The other color to merge.
   * @param weight The weight of the other color [0,1].
   */
  void merge(const Color& other, float weight = 0.5f);

  /**
   * @brief Hash function for colors to use colors as keys in maps, sets, etc.
   */
  struct Hash {
    uint32_t operator()(const Color& color) const {
      return color.r + (color.g << 8) + (color.b << 16) + (color.a << 24);
    }
  };

  // Presets.
  static Color black() { return Color(0, 0, 0); }
  static Color white() { return Color(255, 255, 255); }
  static Color red() { return Color(255, 0, 0); }
  static Color green() { return Color(0, 255, 0); }
  static Color blue() { return Color(0, 0, 255); }
  static Color yellow() { return Color(255, 255, 0); }
  static Color orange() { return Color(255, 127, 0); }
  static Color purple() { return Color(127, 0, 255); }
  static Color cyan() { return Color(0, 255, 255); }
  static Color magenta() { return Color(255, 0, 255); }
  static Color pink() { return Color(255, 127, 127); }
  static Color random();

  // Conversions.
  static Color fromHSV(float hue, float saturation, float value);
  static Color fromHLS(float hue, float luminance, float saturation);

  // Color maps.
  /**
   * @brief Generate a gray-scale color.
   * @param value The value of the gray [0,1, where 0 is black and 1 is white].
   */
  static Color gray(float value = 0.5f);

  /**
   * @brief Generate a color based on a quality value.
   * @param value The quality value [0,1], where 0 is red, 0.5 is yellow, and 1 is
   * green.
   */
  static Color quality(float value);

  /**
   * @brief Generate a color based on a spectrum of colors.
   * @param value The spectrum value [0,1], where 0 is the first color and 1 is the
   * last color.
   * @param colors The list of colors in the spectrum. Colors are assumed equidistant.
   */
  static Color spectrum(float value, const std::vector<Color>& colors);

  /**
   * @brief Generate a color based on a ironbow value.
   * @param value The temperature value [0,1], where 0 is dark and 1 is light.
   */
  static Color ironbow(float value);

  /**
   * @brief Generate a color based on a rainbow value.
   * @param value The rainbow value [0,1], where 0 is red, 0.5 is green, and 1 is
   * blue.
   */
  static Color rainbow(float value);

  /**
   * @brief Generate a sequence of never repeating colors in the rainbow spectrum.
   * @param id The id of the color in the sequence.
   * @param ids_per_revolution The number of colors per revolution of the hue.
   */
  static Color rainbowId(size_t id, size_t ids_per_revolution = 16);

 private:
  static const std::vector<Color> ironbow_colors_;
};

}  // namespace spark_dsg