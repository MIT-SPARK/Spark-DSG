#include <spark_dsg/zmq_interface.h>

#include <chrono>
#include <thread>

auto main(int argc, char* argv[]) -> int {
  if (argc < 2) {
    std::cerr << "Invalid arguments! Usage: dsg_repeater GRAPH_PATH [RATE] [ADDRESS]"
              << std::endl;
    return 1;
  }

  const std::string dsg_path(argv[1]);

  double rate = 5.0;
  if (argc == 3) {
    rate = std::strtod(argv[2], nullptr);
  }

  if (rate <= 0) {
    std::cerr << "Invalid rate: " << rate << "!" << std::endl;
    return 1;
  }

  std::string address = "tcp://127.0.0.1:8001";
  if (argc == 4) {
    address = std::string(argv[3]);
  }

  const auto graph = spark_dsg::DynamicSceneGraph::load(dsg_path);
  if (!graph) {
    std::cerr << "Could not load graph from '" << dsg_path << "'";
    return 1;
  }

  std::cout << "starting repeater @ '" << address << "' for graph from '" << dsg_path
            << "'" << std::endl;
  spark_dsg::ZmqSender sender(address, 2);
  const auto wait_duration = std::chrono::duration<double>(1.0 / rate);
  while (true) {
    sender.send(*graph, true);
    std::this_thread::sleep_for(wait_duration);
  }

  return 0;
}
