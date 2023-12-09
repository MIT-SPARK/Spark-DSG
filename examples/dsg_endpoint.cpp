#include <spark_dsg/zmq_interface.h>

#include <chrono>
#include <thread>

auto main(int argc, char* argv[]) -> int {
  if (argc < 1) {
    std::cerr << "Invalid arguments! Usage: dsg_endpoint [RATE] [ADDRESS]" << std::endl;
    return 1;
  }

  double rate = 5.0;
  if (argc == 2) {
    rate = std::strtod(argv[2], nullptr);
  }

  if (rate <= 0) {
    std::cerr << "Invalid rate: " << rate << "!" << std::endl;
    return 1;
  }

  std::string address = "tcp://127.0.0.1:8001";
  if (argc == 3) {
    address = std::string(argv[3]);
  }

  std::cout << "starting endpoint @ '" << address << "'" << std::endl;
  spark_dsg::ZmqReceiver receiver(address, 2);
  const auto wait_duration_s = std::chrono::duration<double>(1.0 / rate);
  const auto wait_duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(wait_duration_s);
  while (true) {
    if (!receiver.recv(wait_duration_ms.count(), true)) {
      continue;
    }

    auto graph = receiver.graph();
    if (!graph) {
      continue;
    }

    const auto stamp = std::chrono::system_clock::now().time_since_epoch();
    std::cout << "Got graph with " << graph->numNodes() << " nodes and "
              << graph->numEdges() << " edges @ " << stamp.count() << " [ns]"
              << std::endl;
  }

  return 0;
}
