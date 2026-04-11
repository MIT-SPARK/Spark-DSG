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
#include "spark_dsg/python/bounding_box.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <spark_dsg/bounding_box.h>
#include <spark_dsg/bounding_box_extraction.h>

namespace spark_dsg::python {

namespace py = pybind11;
using namespace py::literals;

using bounding_box::BoxResult2D;

void init_bounding_box(py::module_& m) {
  py::enum_<BoundingBox::Type>(m, "BoundingBoxType")
      .value("INVALID", BoundingBox::Type::INVALID)
      .value("AABB", BoundingBox::Type::AABB)
      .value("OBB", BoundingBox::Type::OBB)
      .value("RAABB", BoundingBox::Type::RAABB);

  py::class_<BoundingBox>(m, "BoundingBox")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3f&>())
      .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&>())
      .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&, float>())
      .def(py::init([](BoundingBox::Type type,
                       const Eigen::Vector3f& dim,
                       const Eigen::Vector3f& pos,
                       const Eigen::Matrix3f& trans) { return BoundingBox(type, dim, pos, trans); }))
      .def_readwrite("type", &BoundingBox::type)
      .def_readwrite("dimensions", &BoundingBox::dimensions)
      .def_readwrite("world_P_center", &BoundingBox::world_P_center)
      .def_readwrite("world_R_center", &BoundingBox::world_R_center)
      .def("is_valid", &BoundingBox::isValid)
      .def("volume", &BoundingBox::volume)
      .def("has_rotation", &BoundingBox::hasRotation)
      .def("corners", &BoundingBox::corners)
      .def("contains", py::overload_cast<const Eigen::Vector3f&>(&BoundingBox::contains, py::const_))
      .def("intersects", &BoundingBox::intersects)
      .def("compute_iou", &BoundingBox::computeIoU, "other"_a, "samples"_a = 1000)
      .def("approx_iou", &BoundingBox::computeIoUApprox, "other"_a, "samples"_a = 1000)
      .def_property_readonly("min", [](const BoundingBox& box) { return box.pointToWorldFrame(-box.dimensions / 2); })
      .def_property_readonly("max", [](const BoundingBox& box) { return box.pointToWorldFrame(box.dimensions / 2); })
      .def("__repr__", [](const BoundingBox& box) {
        std::stringstream ss;
        ss << box;
        return ss.str();
      });

  m.def("get_2d_convex_hull", [](const std::vector<Eigen::Vector3f>& points) {
    BoundingBox::PointVectorAdaptor adaptor(points);
    return bounding_box::get2dConvexHull(adaptor);
  });

  py::class_<BoxResult2D>(m, "BoxResult2D")
      .def_readwrite("center", &BoxResult2D::center)
      .def_readwrite("dims", &BoxResult2D::dims)
      .def_readwrite("yaw", &BoxResult2D::yaw)
      .def("__repr__", [](const BoxResult2D& box) {
        std::stringstream ss;
        const Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
        ss << "BoxResult2d<(center=" << box.center.format(fmt) << ", dims=" << box.dims.format(fmt)
           << ", yaw=" << box.yaw << ")>";
        return ss.str();
      });

  m.def(
      "get_min_2d_box",
      [](const std::vector<Eigen::Vector3f>& points, const std::list<size_t>& hull) -> std::optional<BoxResult2D> {
        BoundingBox::PointVectorAdaptor adaptor(points);
        const auto result = bounding_box::getMin2DBox(adaptor, hull);
        return !result.min_area ? std::nullopt : std::optional<BoxResult2D>(result);
      },
      "points"_a,
      "hull"_a = std::list<size_t>{});
}

}  // namespace spark_dsg::python
