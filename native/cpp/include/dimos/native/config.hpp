// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Strict config access over the `config` object the coordinator sends on stdin.
// Mirrors the Rust SDK's contract, minus the derive macro: Python owns every
// default and always sends every field, so the C++ side never fills anything in.
//
//   - require<T>(key)          every field must be present (missing => error)
//   - enforce_all_consumed()   no unknown fields (extra key => error)
//
// Together these are the Rust runtime's one-to-one key check: the set of fields
// the module reads must exactly equal the set of fields Python sent.

#pragma once

#include <nlohmann/json.hpp>

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace dimos::native {

class Config {
public:
    /// `obj` is the `config` value from the stdin JSON. A JSON null (a module
    /// with no config) is treated as an empty object.
    explicit Config(nlohmann::json obj) : obj_(std::move(obj)) {
        if (obj_.is_null()) {
            obj_ = nlohmann::json::object();
        }
        if (!obj_.is_object()) {
            throw std::runtime_error(std::string("config must be a JSON object, got ") +
                                     obj_.type_name());
        }
        for (auto it = obj_.begin(); it != obj_.end(); ++it) {
            keys_.insert(it.key());
        }
    }

    /// Read a required field. Throws if absent or not convertible to `T`.
    /// Records the field as consumed for enforce_all_consumed().
    template <class T>
    T require(const std::string& key) {
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            throw std::runtime_error("config: missing required field '" + key + "'");
        }
        consumed_.insert(key);
        try {
            return it->get<T>();
        } catch (const std::exception& e) {
            throw std::runtime_error("config: field '" + key + "' has the wrong type: " +
                                     e.what());
        }
    }

    /// Read a required numeric field and check `min <= value <= max`.
    template <class T>
    T require_in_range(const std::string& key, T min, T max) {
        T value = require<T>(key);
        if (value < min || value > max) {
            throw std::runtime_error("config: field '" + key + "' out of range [" +
                                     std::to_string(min) + ", " + std::to_string(max) +
                                     "], got " + std::to_string(value));
        }
        return value;
    }

    /// Throw if any field Python sent was never read. This is the deny-unknown
    /// half of the one-to-one check and surfaces both typos and dead config.
    void enforce_all_consumed() const {
        std::vector<std::string> unexpected;
        for (const std::string& key : keys_) {
            if (consumed_.find(key) == consumed_.end()) {
                unexpected.push_back(key);
            }
        }
        if (!unexpected.empty()) {
            std::string msg = "config: unexpected field(s):";
            for (const std::string& key : unexpected) {
                msg += " '" + key + "'";
            }
            throw std::runtime_error(msg);
        }
    }

    bool empty() const { return obj_.empty(); }

private:
    nlohmann::json obj_;
    std::set<std::string> keys_;
    std::set<std::string> consumed_;
};

}  // namespace dimos::native
