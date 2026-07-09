// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "dimos/native/module.hpp"
#include "dimos/native/transport.hpp"

using namespace dimos::native;
using Bytes = std::vector<uint8_t>;

namespace {

// Mock transport that records publishes, lets tests inject inbound messages,
// and can wedge one channel's publish to test head-of-line isolation.
struct MockTransport : Transport {
    std::mutex m;
    std::vector<std::pair<std::string, Bytes>> published;
    std::unordered_map<std::string, Dispatch> subs;
    std::string qos;
    std::atomic<bool> block_enabled{false};
    std::string block_channel;
    std::atomic<bool> release{false};

    void publish(const std::string& channel, Bytes data) override {
        if (block_enabled.load() && channel == block_channel) {
            while (!release.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        std::lock_guard<std::mutex> lock(m);
        published.emplace_back(channel, std::move(data));
    }
    void subscribe(const std::string& channel, Dispatch on_msg) override {
        std::lock_guard<std::mutex> lock(m);
        subs[channel] = std::move(on_msg);
    }
    void set_publisher_qos(const std::string& qos_json) override { qos = qos_json; }

    void deliver(const std::string& channel, const Bytes& bytes) {
        Dispatch cb;
        {
            std::lock_guard<std::mutex> lock(m);
            cb = subs.at(channel);
        }
        cb(bytes.data(), bytes.size());
    }
    bool has_published(const std::string& channel) {
        std::lock_guard<std::mutex> lock(m);
        for (const auto& p : published) {
            if (p.first == channel) {
                return true;
            }
        }
        return false;
    }
};

Bytes identity_decode(const uint8_t* d, std::size_t n) { return Bytes(d, d + n); }
Bytes identity_encode(const Bytes& v) { return v; }

template <class F>
bool wait_until(F cond, std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!cond()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

std::vector<std::thread> start_workers(Builder& builder, Transport& transport) {
    std::vector<std::thread> workers;
    for (const auto& queue : builder.publish_queues()) {
        workers.emplace_back(publish_worker_loop, queue.get(), &transport);
    }
    return workers;
}

void stop_workers(Builder& builder, std::vector<std::thread>& workers) {
    for (const auto& queue : builder.publish_queues()) {
        queue->stop();
    }
    for (std::thread& w : workers) {
        w.join();
    }
}

}  // namespace

TEST_CASE("an inbound message routes through a handler to an output") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"data", "/data"}, {"out", "/out"}}, &notifier);

    Bytes received;
    Output<Bytes> out = builder.output<Bytes>("out", identity_encode);
    builder.input<Bytes>("data", identity_decode, [&](Bytes m) {
        received = m;
        out.publish(m);
    });

    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    auto workers = start_workers(builder, transport);

    transport.deliver("/data", {1, 2, 3});
    for (InputPort* port : builder.input_ports()) {
        port->drain_one();
    }

    CHECK(received == Bytes{1, 2, 3});
    CHECK(wait_until([&] { return transport.has_published("/out"); }));

    stop_workers(builder, workers);
}

TEST_CASE("topic_for maps declared ports and falls back to /port") {
    Notifier notifier;
    Builder builder({{"cmd_vel", "/robot/cmd_vel"}}, &notifier);
    CHECK(builder.topic_for("cmd_vel") == "/robot/cmd_vel");
    CHECK(builder.topic_for("unmapped") == "/unmapped");
}

TEST_CASE("a full input queue drops newest and caps at capacity") {
    Notifier notifier;
    Builder builder({}, &notifier);
    builder.input<Bytes>("data", identity_decode, [](Bytes) {});

    Dispatch dispatch = builder.routes()[0].second;
    for (std::size_t i = 0; i < kInputQueueCapacity + 10; ++i) {
        uint8_t byte = 1;
        dispatch(&byte, 1);
    }

    InputPort* port = builder.input_ports()[0];
    std::size_t drained = 0;
    while (port->drain_one()) {
        ++drained;
    }
    CHECK(drained == kInputQueueCapacity);
}

TEST_CASE("a blocked publish channel does not stall a sibling channel") {
    MockTransport transport;
    transport.block_channel = "/block";
    transport.block_enabled.store(true);

    Notifier notifier;
    Builder builder({{"block_out", "/block"}, {"fast_out", "/fast"}}, &notifier);
    Output<Bytes> block_out = builder.output<Bytes>("block_out", identity_encode);
    Output<Bytes> fast_out = builder.output<Bytes>("fast_out", identity_encode);

    auto workers = start_workers(builder, transport);

    block_out.publish({1});  // wedges the /block worker inside transport.publish
    fast_out.publish({2});

    CHECK(wait_until([&] { return transport.has_published("/fast"); }));
    CHECK_FALSE(transport.has_published("/block"));

    transport.release.store(true);
    stop_workers(builder, workers);
}

TEST_CASE("parse_stdin_config extracts topics, config, and qos") {
    StdinConfig p = parse_stdin_config(
        R"({"topics":{"data":"/d"},"config":{"x":1},"qos":{"/d":{"reliability":"reliable"}}})");
    CHECK(p.topics.at("data") == "/d");
    CHECK(p.config.at("x") == 1);
    CHECK(p.qos.find("reliable") != std::string::npos);
}

TEST_CASE("parse_stdin_config tolerates a missing config and qos") {
    StdinConfig p = parse_stdin_config(R"({"topics":{}})");
    CHECK(p.config.is_null());
    CHECK(p.qos.empty());
}

namespace {
struct WaitModule : Module {
    void build(Builder&, Config&) override {}
    void invoke_default_handle() { default_handle(); }
};
}  // namespace

TEST_CASE("default_handle returns promptly once shutdown is requested") {
    shutdown_flag().store(true);
    WaitModule m;
    m.invoke_default_handle();  // no inputs bound; returns because shutdown is set
    shutdown_flag().store(false);
    CHECK(true);
}
