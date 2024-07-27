#pragma once

/** @file */

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/Json.hpp"

namespace volcart
{
/**
 * @class Metadata
 * @author Sean Karlage, Seth Parker
 * @date 10/27/15
 *
 * @brief Generic interface for storing metadata as key/value pairs
 *
 * Internally uses JSON for Modern C++ for easy storage and [de]serialization:
 * https://nlohmann.github.io/json/
 *
 * @ingroup Types
 */
class Metadata
{

public:
    /**@{*/
    /** @brief Default constructor */
    Metadata() = default;

    /**
     * @brief Read a metadata file from disk
     *
     * @throws volcart::IOException
     */
    explicit Metadata(volcart::filesystem::path fileLocation);
    /**@}*/

    /**@{*/
    /** @brief Get the path where the metadata file will be written */
    volcart::filesystem::path path() const { return path_; }

    /** @brief Set the path where the metadata file will be written */
    void setPath(const volcart::filesystem::path& path) { path_ = path; }

    /**
     * @brief Save the metadata file to the stored path
     *
     * @throws volcart::IOException 
     */
    void save() { save(path_); }

    /** @brief Save the metadata file to a specified path */
    void save(const volcart::filesystem::path& path);
    /**@}*/

    /**@{*/
    /** @brief Return whether the given key is defined */
    bool hasKey(const std::string& key) const { return json_.count(key) > 0; }

    /** @brief Get a metadata value by key
     *
     * Throws an std::runtime_error if the key is not set.
     *
     * @tparam T Value return type. JSON library will attempt to convert to the
     * specified type.
     */
    template <typename T>
    T get(const std::string& key) const
    {
        if (json_.find(key) == json_.end()) {
            auto msg = "could not find key '" + key + "' in metadata";
            throw std::runtime_error(msg);
        }
        if (json_[key].is_null()) {
            T val;
            return val;
        }
        return json_[key].get<T>();
    }

    /**
     * @brief Set a metadata key and value
     *
     * @tparam T Value type. JSON library will store using the specified type.
     */
    template <typename T>
    void set(const std::string& key, T value)
    {
        json_[key] = value;
    }

    /**@{*/
    /**
     * @brief Print a string representation of the metadata to std::cout
     *
     * @warning This should only be used for debugging.
     */
    void printString() const { std::cout << json_ << std::endl; }

    /**
     * @brief Print an object representation of the metadata to std::cout
     *
     * @warning This should only be used for debugging.
     */
    void printObject() const { std::cout << json_.dump(4) << std::endl; }
    /**@}*/
protected:
    /** JSON data storage */
    nlohmann::json json_;
    /** Location where the JSON file will be stored*/
    volcart::filesystem::path path_;
};
}  // namespace volcart
