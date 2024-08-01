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

#include <vector>

#include "spark_dsg/color.h"

namespace spark_dsg::colormaps {

/**
 * @brief Generate a gray-scale color.
 * @param value The value of the gray [0,1, where 0 is black and 1 is white].
 */
Color gray(float value = 0.5f);

/**
 * @brief Generate a color based on a quality value.
 * @param value The quality value [0,1], where 0 is red, 0.5 is yellow, and 1 is
 * green.
 */
Color quality(float value);

/**
 * @brief Generate a color based on a spectrum of colors.
 * @param value The spectrum value [0,1], where 0 is the first color and 1 is the
 * last color.
 * @param colors The list of colors in the spectrum. Colors are assumed equidistant.
 */
Color spectrum(float value, const std::vector<Color>& colors);

/**
 * @brief Interpolate between two colors in HLS space
 * @param value A weight in [0,1] where 0 is the first color and 1 is the last
 * @param start The color corresponding to 0
 * @param end The color corresponding to 1
 */
Color hls(float value, const Color& start, const Color& end);

/**
 * @brief Interpolate between two colors in HLS space
 * @param value A weight in [0,1] where 0 is the first color and 1 is the last
 * @param start The HLS color corresponding to 0
 * @param end The HLS color corresponding to 1
 */
Color hls(float value,
          const std::array<float, 3>& start,
          const std::array<float, 3>& end);

/**
 * @brief Generate a color based on a ironbow value.
 * @param value The temperature value [0,1], where 0 is dark and 1 is light.
 */
Color ironbow(float value);

/**
 * @brief Generate a color based on a rainbow value.
 * @param value The rainbow value [0,1], where 0 is red, 0.5 is green, and 1 is
 * blue.
 */
Color rainbow(float value);

/**
 * @brief Generate a sequence of never repeating colors in the rainbow spectrum.
 * @param id The id of the color in the sequence.
 * @param ids_per_revolution The number of colors per revolution of the hue.
 */
Color rainbowId(size_t id, size_t ids_per_revolution = 16);

/**
 * @brief Pick a color from
 * https://colorbrewer2.org/#type=qualitative&scheme=Paired&n=12
 * @param id The id of the color in the sequence
 * @note Indices outside of the 0-11 range will get remapped to be inside the range
 */
Color colorbrewer(size_t id);

}  // namespace spark_dsg::colormaps
