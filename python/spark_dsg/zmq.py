# py::class_<ZmqSender>(module, "DsgSender")
# .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
# .def("send", &ZmqSender::send, "graph"_a, "include_mesh"_a = false);

# py::class_<ZmqReceiver>(module, "DsgReceiver")
# .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
# .def("recv", &ZmqReceiver::recv, "timeout_ms"_a, "recv_all"_a = false)
# .def_property_readonly("graph", [](const ZmqReceiver& receiver) {
# if (!receiver.graph()) {
# throw pybind11::value_error("no graph received yet");
# }
# return receiver.graph();
# });

# py::class_<ZmqGraph>(module, "ZmqGraph")
# .def(py::init<const std::string&, size_t, size_t>(),
# "url"_a,
# "num_threads"_a = 1,
# "poll_time_ms"_a = 100)
# .def_property_readonly(
# "has_change", [](const ZmqGraph& zmq_graph) { return zmq_graph.hasChange(); })
# .def_property_readonly(
# "graph", [](const ZmqGraph& zmq_graph) { return zmq_graph.graph(); });

# module.def("version",
# []() { return spark_dsg::io::FileHeader::current().version.toString(); });
