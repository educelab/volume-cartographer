#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

#include "vc/core/vc_defines.hpp"
#include "vc/external/json.hpp"

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

    /** @brief Read a metadata file from disk */
    explicit Metadata(boost::filesystem::path fileLocation);
    /**@}*/

    /**@{*/
    /** @brief Get the path where the metadata file will be written */
    boost::filesystem::path path() const { return path_; }

    /** @brief Set the path where the metadata file will be written */
    void setPath(const boost::filesystem::path& path) { path_ = path; }

    /** @brief Save the metadata file to the stored path */
    void save() { save(path_); }

    /** @brief Save the metadata file to a specified path */
    void save(const boost::filesystem::path& path);
    /**@}*/

    /**@{*/
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
    /**@}*/

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
    boost::filesystem::path path_;
};
}
