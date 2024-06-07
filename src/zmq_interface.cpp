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
#include "spark_dsg/zmq_interface.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <zmq.hpp>

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/serialization/graph_binary_serialization.h"

namespace spark_dsg {

class ZmqContextHolder {
 public:
  zmq::context_t& context() { return *context_; }

  void set_threads(size_t num_threads) {
    zmq_ctx_set(context_->operator void*(), ZMQ_IO_THREADS, num_threads);
  }

  static ZmqContextHolder& instance() {
    if (!instance_) {
      instance_.reset(new ZmqContextHolder());
    }
    return *instance_;
  }

 private:
  ZmqContextHolder() { context_.reset(new zmq::context_t(2)); }

  static std::unique_ptr<ZmqContextHolder> instance_;
  std::unique_ptr<zmq::context_t> context_;
};

std::unique_ptr<ZmqContextHolder> ZmqContextHolder::instance_;

struct ZmqSender::Detail {
  Detail(const std::string& url, size_t) {
    socket.reset(new zmq::socket_t(ZmqContextHolder::instance().context(), ZMQ_PUB));
    socket->bind(url);
  }

  ~Detail() = default;

  void send(const DynamicSceneGraph& graph, bool include_mesh) {
    std::vector<uint8_t> buffer;
    io::binary::writeGraph(graph, buffer, include_mesh);

    // TODO(nathan) it'd be nice if we could avoid the memcpy
    zmq::message_t msg(buffer.data(), buffer.size());
#if ZMQ_VERSION < ZMQ_MAKE_VERSION(4, 3, 1)
    socket->send(msg, 0);
#else
    socket->send(msg, zmq::send_flags::none);
#endif
  }

  std::unique_ptr<zmq::socket_t> socket;
};

ZmqSender::ZmqSender(const std::string& url, size_t num_threads)
    : internals_(new ZmqSender::Detail(url, num_threads)) {}

ZmqSender::~ZmqSender() {}

void ZmqSender::send(const DynamicSceneGraph& graph, bool include_mesh) {
  internals_->send(graph, include_mesh);
}

struct ZmqReceiver::Detail {
  Detail(const std::string& url, size_t, bool conflate) {
    socket.reset(new zmq::socket_t(ZmqContextHolder::instance().context(), ZMQ_SUB));
    socket->connect(url);
    socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    if (conflate) {
      int conflate_flag = 1;
      socket->setsockopt(ZMQ_CONFLATE, &conflate_flag, sizeof(conflate_flag));
    }
  }

  ~Detail() = default;

  bool recv(size_t timeout_ms) {
    if (!socket->connected()) {
      return false;
    }

    zmq::pollitem_t items[] = {{socket->operator void*(), 0, ZMQ_POLLIN, 0}};
    zmq::poll(&items[0], 1, std::chrono::milliseconds(timeout_ms));

    if (!(items[0].revents & ZMQ_POLLIN)) {
      return false;
    }

    zmq::message_t msg;
#if ZMQ_VERSION < ZMQ_MAKE_VERSION(4, 3, 1)
    socket->recv(&msg);
#else
    const auto ret = socket->recv(msg, zmq::recv_flags::none);
    if (!ret) {
      throw std::runtime_error("zmq internal error: no data received");
    }
#endif

    if (!graph) {
      graph = io::binary::readGraph(static_cast<uint8_t*>(msg.data()), msg.size());
    } else {
      const auto data_ptr = static_cast<uint8_t*>(msg.data());
      io::binary::updateGraph(*graph, data_ptr, msg.size());
    }

    return true;
  }

  std::unique_ptr<zmq::socket_t> socket;
  DynamicSceneGraph::Ptr graph;
};

ZmqReceiver::ZmqReceiver(const std::string& url, size_t num_threads, bool conflate)
    : internals_(new ZmqReceiver::Detail(url, num_threads, conflate)) {}

ZmqReceiver::~ZmqReceiver() {}

bool ZmqReceiver::recv(size_t timeout_ms, bool recv_all) {
  const auto have_data = internals_->recv(timeout_ms);
  if (!have_data || !recv_all) {
    return have_data;
  }

  // spin while we still have messages
  while (internals_->recv(1)) {
  }
  return true;
}

DynamicSceneGraph::Ptr ZmqReceiver::graph() const { return internals_->graph; }

struct ZmqGraph::Detail {
 public:
  Detail(const std::string& url, size_t num_threads, size_t poll_time_ms = 100)
      : poll_time_ms_(poll_time_ms),
        has_change_(false),
        receiver_(url, num_threads),
        recv_thread_(new std::thread(&Detail::receiveLoop, this)) {}

  ~Detail() {
    should_shutdown_ = true;
    if (recv_thread_) {
      recv_thread_->join();
      recv_thread_.reset();
    }
  }

  bool hasChange() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_change_;
  }

  DynamicSceneGraph::Ptr graph() const {
    std::lock_guard<std::mutex> lock(mutex_);
    has_change_ = false;
    if (!graph_) {
      return nullptr;
    }

    return graph_->clone();
  }

 private:
  void receiveLoop() {
    while (!should_shutdown_) {
      const auto new_data = receiver_.recv(poll_time_ms_, true);
      if (!new_data) {
        continue;
      }

      std::lock_guard<std::mutex> lock(mutex_);
      has_change_ = true;
      graph_ = receiver_.graph();
    }
  }

  const size_t poll_time_ms_;
  std::atomic<bool> should_shutdown_;
  mutable bool has_change_;
  ZmqReceiver receiver_;

  mutable std::mutex mutex_;
  DynamicSceneGraph::Ptr graph_;
  std::unique_ptr<std::thread> recv_thread_;
};

ZmqGraph::ZmqGraph(const std::string& url, size_t num_threads, size_t poll_time_ms)
    : internals_(new ZmqGraph::Detail(url, num_threads, poll_time_ms)) {}

ZmqGraph::~ZmqGraph() {}

bool ZmqGraph::hasChange() const { return internals_->hasChange(); }

DynamicSceneGraph::Ptr ZmqGraph::graph() const { return internals_->graph(); }

}  // namespace spark_dsg
