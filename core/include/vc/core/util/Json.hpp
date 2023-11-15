#pragma once

/**
 * @file Json.hpp
 * JSON serialization/deserialization type specializations.
 */

#include <nlohmann/json.hpp>

NLOHMANN_JSON_NAMESPACE_BEGIN
/* cv::Vec */
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

/* cv::Mat_ */
template <typename T>
struct adl_serializer<cv::Mat_<T>> {
    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void to_json(json& j, const cv::Mat_<T>& m)
    {
        for (int r = 0; r < m.rows; r++) {
            json row;
            for (int c = 0; c < m.cols; c++) {
                row.push_back(m(r, c));
            }
            j.push_back(row);
        }
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Must be exact signature
    static void from_json(const json& j, cv::Mat_<T>& m)
    {
        auto rows = j.size();
        auto cols = j[0].size();
        m = cv::Mat_<T>(rows, cols);
        for (int r = 0; r < m.rows; r++) {
            for (int c = 0; c < m.cols; c++) {
                m(r, c) = j[r][c];
            }
        }
    }
};
NLOHMANN_JSON_NAMESPACE_END