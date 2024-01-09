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
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace spark_dsg {

struct Color {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t a = 255;

  Color() = default;
  Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) : r(r), g(g), b(b), a(a) {}
};

class Mesh {
 public:
  using Ptr = std::shared_ptr<Mesh>;
  using Pos = Eigen::Vector3f;
  using Face = std::array<size_t, 3>;
  using Timestamp = uint64_t;
  using Label = uint32_t;

  // Constructors.
  Mesh(bool has_colors = true, bool has_timestamps = true, bool has_labels = true);

  virtual ~Mesh();

  // Assignment operators and constructors
  Mesh& operator=(const Mesh& other);
  Mesh& operator=(Mesh&& other) noexcept;
  Mesh(const Mesh& other);
  Mesh(Mesh&& other) noexcept;

  /**
   * @brief Check whether the mesh is empty
   * @returns Returns true if the mesh has vertices and faces
   */
  bool empty() const;

  /**
   * @brief Get total number of vertices
   */
  size_t numVertices() const;

  /**
   * @brief Get total number of vertices
   */
  size_t numFaces() const;

  /**
   * @brief Set mesh vertex size
   * @param size New size of mesh vertices
   */
  void resizeVertices(size_t size);

  /**
   * @brief Set mesh face size
   */
  void resizeFaces(size_t size);

  /**
   * @brief Copy mesh (allowing for derived classes)
   */
  virtual Mesh::Ptr clone() const;

  /**
   * @brief Get current position of vertex
   */
  const Pos& pos(size_t index) const;

  /**
   * @brief Set position of vertex
   */
  void setPos(size_t index, const Mesh::Pos& pos);

  /**
   * @brief Get current color of vertex
   */
  const Color& color(size_t index) const;

  /**
   * @brief Set current color of vertex
   */
  void setColor(size_t index, const Color& color);

  /**
   * @brief Get current timestamp
   */
  Timestamp timestamp(size_t index) const;

  /**
   * @brief Set current timestamp
   */
  void setTimestamp(size_t index, Timestamp label);

  /**
   * @brief Get last seen timestamp.
   */
  Timestamp lastSeenTimestamp(size_t index) const;

  /**
   * @brief Set last seen timestamp.
   */
  void setLastSeenTimestamp(size_t index, Timestamp timestamp);

  /**
   * @brief Get current label
   */
  Label label(size_t index) const;

  /**
   * @brief Get current label
   */
  void setLabel(size_t index, Label label);

  /**
   * @brief Get a face
   */
  const Face& face(size_t index) const;

  /**
   * @brief Get a face (non-const)
   */
  Face& face(size_t index);

 public:
  /**
   * @brief Get JSON string representing mesh
   * @returns JSON string representing mesh
   */
  std::string serializeToJson() const;

  /**
   * @brief Save mesh to binary representation
   */
  void serializeToBinary(std::vector<uint8_t>& buffer) const;

  /**
   * @brief Save the mesh to file.
   *
   * Uses the extension to determine serialization strategy
   *
   * @param filepath Filepath to save graph to.
   */
  void save(std::string filepath) const;

  /**
   * @brief parse mesh from JSON string
   * @param contents JSON string to parse
   * @returns Resulting parsed mesh
   */
  static Ptr deserializeFromJson(const std::string& contents);

  /**
   * @brief parse graph from binary data
   * @param buffer start position of binary data to parse
   * @param length length of binary data to parse
   * @returns Resulting parsed scene graph
   */
  static Ptr deserializeFromBinary(const uint8_t* const buffer, size_t length);

  /**
   * @brief parse mesh from binary or JSON file
   * @param filepath Complete path to file to read, including extension.
   * @returns Resulting parsed mesh
   */
  static Ptr load(std::string filepath);

 public:
  const bool has_colors;
  const bool has_timestamps;
  const bool has_labels;
  std::vector<Pos> points;
  std::vector<Color> colors;
  std::vector<Timestamp> stamps;
  std::vector<Label> labels;
  std::vector<Face> faces;

  // Additional last seen timestamps for khronos, where 'stamps' denotes the first seen
  // stamps.
  std::vector<Timestamp> last_seen_stamps;
};

}  // namespace spark_dsg
