#include "kimera_dsg/serialization_helpers.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using nlohmann::json;

namespace pcl {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(pcl::PointXYZRGBA, x, y, z, r, g, b, a);
}

namespace pcl {

void to_json(json& j, const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  j["vertices"] = json::array();
  for (const auto& vertex : vertices) {
    json vertex_json = vertex;
    j.at("vertices").push_back(vertex_json);
  }

  j["faces"] = json::array();
  for (const auto& face : mesh.polygons) {
    j.at("faces").push_back(face.vertices);
  }
}

void from_json(const json& j, pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  for (const auto& vertex : j.at("vertices")) {
    vertices.push_back(vertex.get<pcl::PointXYZRGBA>());
  }

  pcl::toPCLPointCloud2(vertices, mesh.cloud);

  for (const auto& face : j.at("faces")) {
    pcl::Vertices face_indices;
    face_indices.vertices = face.get<std::vector<std::uint32_t>>();
    mesh.polygons.push_back(face_indices);
  }
}

}  // namespace pcl

namespace kimera {

void to_json(json& j, const BoundingBox& b) {
  j = json{{"type", b.type},
           {"min", b.min},
           {"max", b.max},
           {"world_P_center", b.world_P_center},
           {"world_R_center", b.world_R_center}};
}

void from_json(const json& j, BoundingBox& b) {
  b.type = j.at("type").get<BoundingBox::Type>();
  if (b.type == BoundingBox::Type::INVALID) {
    return;
  }

  b.min = j.at("min").get<Eigen::Vector3f>();
  b.max = j.at("max").get<Eigen::Vector3f>();
  b.world_P_center = j.at("world_P_center").get<Eigen::Vector3f>();
  b.world_R_center = j.at("world_R_center").get<Eigen::Quaternionf>();
}

}  // namespace kimera
