// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>
#include <vector>

#include "dimos/native/config.hpp"

using dimos::native::Config;
using nlohmann::json;

TEST_CASE("require reads typed fields") {
    Config cfg(json{{"freq", 10.5}, {"name", "lidar"}, {"count", 3}, {"on", true}});
    CHECK(cfg.require<double>("freq") == doctest::Approx(10.5));
    CHECK(cfg.require<std::string>("name") == "lidar");
    CHECK(cfg.require<int>("count") == 3);
    CHECK(cfg.require<bool>("on") == true);
}

TEST_CASE("require reads list fields") {
    Config cfg(json{{"extrinsic", {1.0, 2.0, 3.0}}});
    auto v = cfg.require<std::vector<double>>("extrinsic");
    REQUIRE(v.size() == 3);
    CHECK(v[2] == doctest::Approx(3.0));
}

TEST_CASE("a missing required field is rejected and named") {
    Config cfg(json{{"freq", 10.0}});
    try {
        cfg.require<double>("host_ip");
        FAIL("expected missing field to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("missing required field") != std::string::npos);
        CHECK(msg.find("host_ip") != std::string::npos);
    }
}

TEST_CASE("a wrong-typed field is rejected and named") {
    Config cfg(json{{"freq", "not_a_number"}});
    try {
        cfg.require<double>("freq");
        FAIL("expected wrong-type field to throw");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("freq") != std::string::npos);
        CHECK(std::string(e.what()).find("wrong type") != std::string::npos);
    }
}

TEST_CASE("require_in_range accepts in-range and rejects out-of-range") {
    Config cfg(json{{"a", 5}, {"b", 99}});
    CHECK(cfg.require_in_range<int>("a", 0, 10) == 5);
    try {
        cfg.require_in_range<int>("b", 0, 10);
        FAIL("expected out-of-range value to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("out of range") != std::string::npos);
        CHECK(msg.find("b") != std::string::npos);
    }
}

TEST_CASE("enforce_all_consumed passes once every field is read") {
    Config cfg(json{{"a", 1}, {"b", 2}});
    cfg.require<int>("a");
    cfg.require<int>("b");
    cfg.enforce_all_consumed();  // must not throw
    CHECK(true);
}

TEST_CASE("enforce_all_consumed rejects unknown fields and names them") {
    Config cfg(json{{"a", 1}, {"typo", 2}});
    cfg.require<int>("a");
    try {
        cfg.enforce_all_consumed();
        FAIL("expected unconsumed field to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("unexpected field") != std::string::npos);
        CHECK(msg.find("typo") != std::string::npos);
    }
}

TEST_CASE("a null config behaves as empty") {
    Config cfg(json(nullptr));
    CHECK(cfg.empty());
    cfg.enforce_all_consumed();  // nothing sent, nothing read: fine
    CHECK_THROWS_AS(cfg.require<int>("anything"), std::runtime_error);
}

TEST_CASE("a non-object config is rejected") {
    CHECK_THROWS_AS(Config(json(42)), std::runtime_error);
    CHECK_THROWS_AS(Config(json::array({1, 2})), std::runtime_error);
}
