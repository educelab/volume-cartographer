#pragma once

/** @file */

#include <optional>

#include <nlohmann/json.hpp>

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
    explicit Metadata(const filesystem::path& fileLocation);
    /**@}*/

    /**@{*/
    /** @brief Get the path where the metadata file will be written */
    [[nodiscard]] auto path() const -> filesystem::path;

    /** @brief Set the path where the metadata file will be written */
    void setPath(const filesystem::path& path);

    /**
     * @brief Save the metadata file to the stored path
     *
     * @throws IOException
     */
    void save() const;

    /** @brief Save the metadata file to a specified path */
    void save(const filesystem::path& path) const;
    /**@}*/

    /**@{*/
    /** @brief Return whether the given key is defined */
    [[nodiscard]] auto hasKey(const std::string& key) const -> bool;

    /**
     * @brief Get a metadata value by key
     *
     * If the stored value is convertible to T, the returned optional will
     * have an assigned value. If the stored value is `null`, the returned
     * optional will have no value. This can be checked with
     * `result.has_value()`, and the value can be retrieved with
     * `result.value()`.
     *
     * @throws std::runtime_error If the key is not present.
     *
     * @tparam T Value return type. JSON library will attempt to convert to the
     * specified type.
     */
    template <typename T>
    auto get(const std::string& key) const -> std::optional<T>
    {
        if (not json_.contains(key)) {
            const auto msg = "could not find key '" + key + "' in metadata";
            throw std::runtime_error(msg);
        }

        if (json_[key].is_null()) {
            return std::optional<T>();
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
     * @brief Print an object representation of the metadata to std::cout
     *
     * @warning This should only be used for debugging.
     */
    void printObject() const;
    /**@}*/
protected:
    /** JSON data storage */
    nlohmann::json json_;
    /** Location where the JSON file will be stored*/
    filesystem::path path_;
};
}  // namespace volcart
