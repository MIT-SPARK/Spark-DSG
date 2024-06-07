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
#include "zmq_bindings.h"

#include <spark_dsg/zmq_interface.h>

using namespace spark_dsg;
using namespace pybind11::literals;

namespace py = pybind11;

#if INCLUDE_ZMQ()
void add_zmq_bindings(pybind11::module_& module) {
  py::class_<ZmqSender>(module, "DsgSender")
      .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
      .def("send", &ZmqSender::send, "graph"_a, "include_mesh"_a = false);

  py::class_<ZmqReceiver>(module, "DsgReceiver")
      .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
      .def("recv", &ZmqReceiver::recv, "timeout_ms"_a, "recv_all"_a = false)
      .def_property_readonly("graph", [](const ZmqReceiver& receiver) {
        if (!receiver.graph()) {
          throw pybind11::value_error("no graph received yet");
        }
        return receiver.graph();
      });

  py::class_<ZmqGraph>(module, "ZmqGraph")
      .def(py::init<const std::string&, size_t, size_t>(),
           "url"_a,
           "num_threads"_a = 1,
           "poll_time_ms"_a = 100)
      .def_property_readonly(
          "has_change", [](const ZmqGraph& zmq_graph) { return zmq_graph.hasChange(); })
      .def_property_readonly(
          "graph", [](const ZmqGraph& zmq_graph) { return zmq_graph.graph(); });
}
#else
void add_zmq_bindings(pybind11::module_&) {}
#endif
