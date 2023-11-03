#pragma once

/**
 * @file Json.hpp
 * JSON serialization/deserialization type specializations.
 */

#include <nlohmann/json.hpp>

/** cv::Vec */
NLOHMANN_JSON_NAMESPACE_BEGIN
template <typename T, int Cn>
struct adl_serializer<cv::Vec<T, Cn>> {
    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void to_json(json& j, const cv::Vec<T, Cn>& v)
    {
        for (int i = 0; i < Cn; i++) {
            j.push_back(v[i]);
        }
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void from_json(const json& j, cv::Vec<T, Cn>& v)
    {
        for (int i = 0; i < Cn; i++) {
            v[i] = j.at(i).get<T>();
        }
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void to_json(ordered_json& j, const cv::Vec<T, Cn>& v)
    {
        for (int i = 0; i < Cn; i++) {
            j.push_back(v[i]);
        }
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void from_json(const ordered_json& j, cv::Vec<T, Cn>& v)
    {
        for (int i = 0; i < Cn; i++) {
            v[i] = j.at(i).get<T>();
        }
    }
};
NLOHMANN_JSON_NAMESPACE_END