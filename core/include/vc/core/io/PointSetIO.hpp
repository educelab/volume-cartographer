#pragma once

/** @file */

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/util/String.hpp"

namespace volcart
{

/** @brief IO Mode for file readers/writers
 *
 * @ingroup IO
 */
enum class IOMode { ASCII = 0, BINARY };

/**
 * @class PointSetIO
 * @author Sean Karlage
 * @brief Read and write PointSet and OrderedPointSet
 *
 * PointSet and OrderedPointSet files always begin with an ASCII header. Point
 * information is then encoded in either ASCII or binary, as determined at
 * time of write.
 *
 * @ingroup IO
 *
 * @see volcart::PointSet
 * @see volcart::OrderedPointSet
 */
template <typename T>
class PointSetIO
{
public:
    /** @brief PointSet file header information */
    struct Header {
        std::size_t width{0};
        std::size_t height{0};
        std::size_t size{0};
        std::size_t dim{0};
        bool ordered{false};
        std::string type;
    };

    /**@{*/
    /**
     * @brief Read OrderedPointSet from file
     *
     * The IOMode should match the encoding type of the file. Point information
     * is expected to be stored in the type and order specified by the template
     * parameter T.
     */
    static OrderedPointSet<T> ReadOrderedPointSet(
        const volcart::filesystem::path& path, IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
            case IOMode::BINARY:
                return ReadOrderedPointSetBinary(path);
            case IOMode::ASCII:
                return ReadOrderedPointSetAscii(path);
            default:
                throw IOException("unsupported IOMode");
        }
    }

    /** @brief Read PointSet from file
     *
     * @copydetails PointSetIO::ReadOrderedPointSet()
     */
    static PointSet<T> ReadPointSet(
        const volcart::filesystem::path& path, IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
            case IOMode::BINARY:
                return ReadPointSetBinary(path);
            case IOMode::ASCII:
                return ReadPointSetAscii(path);
            default:
                throw IOException("unsupported IOMode");
        }
    }
    /**@}*/

    /**@{*/
    /** @brief Write an OrderedPointSet to disk */
    static void WriteOrderedPointSet(
        const volcart::filesystem::path& path,
        const OrderedPointSet<T>& ps,
        IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
            case IOMode::BINARY:
                return WriteOrderedPointSetBinary(path, ps);
            case IOMode::ASCII:
                return WriteOrderedPointSetAscii(path, ps);
        }
    }

    /** @brief Write a PointSet to disk */
    static void WritePointSet(
        const volcart::filesystem::path& path,
        const PointSet<T>& ps,
        IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
            case IOMode::BINARY:
                return WritePointSetBinary(path, ps);
            case IOMode::ASCII:
                return WritePointSetAscii(path, ps);
        }
    }
    /**@}*/

    /**@{*/
    /** @brief Generate a PointSet header string */
    static std::string MakeHeader(PointSet<T> ps)
    {
        std::stringstream ss;
        ss << "size: " << ps.size() << std::endl;
        ss << "dim: " << T::channels << std::endl;
        ss << "ordered: false" << std::endl;

        // Output type information
        ss << "type: ";
        if (std::is_same<typename T::value_type, int>::value) {
            ss << "int" << std::endl;
        } else if (std::is_same<typename T::value_type, float>::value) {
            ss << "float" << std::endl;
        } else if (std::is_same<typename T::value_type, double>::value) {
            ss << "double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw IOException(msg);
        }

        ss << "version: " << PointSet<T>::FORMAT_VERSION << std::endl;
        ss << PointSet<T>::HEADER_TERMINATOR << std::endl;

        return ss.str();
    }
    /** @brief Generate an OrderedPointSet header string */
    static std::string MakeOrderedHeader(OrderedPointSet<T> ps)
    {
        std::stringstream ss;
        ss << "width: " << ps.width() << std::endl;
        ss << "height: " << ps.height() << std::endl;
        ss << "dim: " << T::channels << std::endl;
        ss << "ordered: true" << std::endl;

        // Output type information
        ss << "type: ";
        if (std::is_same<typename T::value_type, int>::value) {
            ss << "int" << std::endl;
        } else if (std::is_same<typename T::value_type, float>::value) {
            ss << "float" << std::endl;
        } else if (std::is_same<typename T::value_type, double>::value) {
            ss << "double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw IOException(msg);
        }

        ss << "version: " << PointSet<T>::FORMAT_VERSION << std::endl;
        ss << PointSet<T>::HEADER_TERMINATOR << std::endl;

        return ss.str();
    }

    /**
     * @brief Parse a PointSet/OrderedPointSet file header
     *
     * The input file stream should be open and pointing at the beginning of
     * the file. Will throw an exception if the ordered parameter does not match
     * the ordereing type of the file.
     */
    static Header ParseHeader(std::ifstream& infile, bool ordered = true)
    {
        // Regexes
        std::regex comments{"^#"};
        std::regex width{"^width"};
        std::regex height{"^height"};
        std::regex dim{"^dim"};
        std::regex ordering{"^ordered"};
        std::regex type{"^type"};
        std::regex size{"^size"};
        std::regex version{"^version"};
        std::regex validTypes{"^(float)|(double)|(int)"};
        std::regex headerTerminator{PointSet<T>::HEADER_TERMINATOR_REGEX};

        Header h;
        std::string line;
        std::vector<std::string> strs;

        while (std::getline(infile, line)) {
            trim(line);
            strs = split(line, ':');
            std::for_each(
                std::begin(strs), std::end(strs), [](auto& s) { trim(s); });

            // Comments: look like:
            // # This is a comment
            //    # This is another comment
            if (std::regex_match(strs[0], comments)) {
                continue;
            }

            // Width
            else if (std::regex_match(strs[0], width)) {
                h.width = std::stoul(strs[1]);
            }

            // Height
            else if (std::regex_match(strs[0], height)) {
                h.height = std::stoul(strs[1]);
            }

            // Size
            else if (std::regex_match(strs[0], size)) {
                h.size = std::stoul(strs[1]);
            }

            // Dim
            else if (std::regex_match(strs[0], dim)) {
                auto parsedDim = std::stoul(strs[1]);
                if (parsedDim != T::channels) {
                    auto msg =
                        "Incorrect dimension read for template specification";
                    throw IOException(msg);
                }
                h.dim = parsedDim;
            }

            // Ordering
            else if (std::regex_match(strs[0], ordering)) {
                to_lower(strs[1]);
                if (strs[1] == "true") {
                    h.ordered = true;
                } else if (strs[1] == "false") {
                    h.ordered = false;
                } else {
                    auto msg = "'ordered' key must have value 'true'/'false'";
                    throw IOException(msg);
                }
            }

            // Type
            else if (std::regex_match(strs[0], type)) {
                if (!std::regex_match(strs[1], validTypes)) {
                    auto msg = "Valid types are int, float, double. Got: '" +
                               strs[1] + "'";
                    throw IOException(msg);
                }

                // type validation
                std::string readerType;
                if (std::is_same_v<typename T::value_type, int>) {
                    readerType = "int";
                } else if (std::is_same_v<typename T::value_type, float>) {
                    readerType = "float";
                } else if (std::is_same_v<typename T::value_type, double>) {
                    readerType = "double";
                } else {
                    throw IOException("unsupported reader type");
                }
                if (strs[1] != readerType) {
                    auto msg = "Type mismatch: vcps filetype '" + strs[1] +
                               "' not compatible with reader type '" +
                               readerType + "'";
                    throw IOException(msg);
                }
                h.type = strs[1];
            }

            // Version
            else if (std::regex_match(strs[0], version)) {
                auto fileVersion = std::stoi(strs[1]);
                if (fileVersion != PointSet<T>::FORMAT_VERSION) {
                    auto msg = "Version mismatch. VCPS file version is " +
                               strs[1] + ", processing version is " +
                               std::to_string(PointSet<T>::FORMAT_VERSION) +
                               ".";
                    throw IOException(msg);
                }
            }

            // End of the header
            else if (std::regex_match(line, headerTerminator)) {
                break;
            }

            // Ignore everything else
            else {
                continue;
            }
            strs.clear();
        }

        // Set size
        if (!ordered && h.width > 0 && h.height > 0) {
            h.size = h.width * h.height;
        }

        // Sanity check. Do we have a valid pointset header?
        if (h.type.empty()) {
            const auto* msg = "Must provide type";
            throw IOException(msg);
        } else if (h.dim == 0) {
            const auto* msg = "Must provide dim";
            throw IOException(msg);
        } else if (!ordered && h.size == 0) {
            const auto* msg = "Unordered pointsets must have a size";
            throw IOException(msg);
        } else if (ordered && (h.width == 0 || h.height == 0)) {
            const auto* msg =
                "Ordered pointsets must have a nonzero width and height";
            throw IOException(msg);
        } else if (ordered && !h.ordered) {
            const auto* msg =
                "Tried to read unordered pointset with ordered PointSetIO";
            throw IOException(msg);
        }

        return h;
    }
    /**@}*/

private:
    /**@{*/
    /** @brief Read an ASCII PointSet */
    static PointSet<T> ReadPointSetAscii(const volcart::filesystem::path& path)
    {
        std::ifstream infile{path.string()};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        // Get header
        auto header = PointSetIO<T>::ParseHeader(infile, false);
        PointSet<T> ps{header.size};

        for (std::size_t i = 0; i < header.size; ++i) {
            std::array<typename T::value_type, T::channels> values;
            for (std::size_t d = 0; d < header.dim; ++d) {
                infile >> values[d];
            }
            ps.push_back(T{values.data()});
        }

        return ps;
    }

    /** @brief Read an ASCII OrderedPointSet */
    static OrderedPointSet<T> ReadOrderedPointSetAscii(
        const volcart::filesystem::path& path)
    {
        std::ifstream infile{path.string()};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        // Get header
        auto header = PointSetIO<T>::ParseHeader(infile, true);
        OrderedPointSet<T> ps{header.width};

        std::vector<T> points;
        points.reserve(header.width);
        for (std::size_t h = 0; h < header.height; ++h) {
            for (std::size_t w = 0; w < header.width; ++w) {
                std::array<typename T::value_type, T::channels> values;
                for (std::size_t d = 0; d < header.dim; ++d) {
                    infile >> values.at(d);
                }
                points.emplace_back(values.data());
            }
            ps.pushRow(points);
            points.clear();
        }

        return ps;
    }

    /** @brief Read a binary PointSet */
    static PointSet<T> ReadPointSetBinary(const volcart::filesystem::path& path)
    {
        std::ifstream infile{path.string(), std::ios::binary};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }
        auto header = PointSetIO<T>::ParseHeader(infile, false);
        PointSet<T> ps{header.size};

        // Size of binary elements to read
        std::size_t typeBytes{};
        if (header.type == "float") {
            typeBytes = sizeof(float);
        } else if (header.type == "double") {
            typeBytes = sizeof(double);
        } else if (header.type == "int") {
            typeBytes = sizeof(int);
        }

        // Read data
        T t;
        for (std::size_t i = 0; i < header.size; ++i) {
            auto nbytes = header.dim * typeBytes;
            infile.read(reinterpret_cast<char*>(t.val), nbytes);
            ps.push_back(t);
        }

        return ps;
    }

    /** @brief Read a binary OrderedPointSet */
    static OrderedPointSet<T> ReadOrderedPointSetBinary(
        const volcart::filesystem::path& path)
    {
        std::ifstream infile{path.string(), std::ios::binary};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }
        auto header = PointSetIO<T>::ParseHeader(infile, true);
        OrderedPointSet<T> ps{header.width};

        // Size of binary elements to read
        std::size_t typeBytes = sizeof(int);
        if (header.type == "float") {
            typeBytes = sizeof(float);
        } else if (header.type == "double") {
            typeBytes = sizeof(double);
        } else if (header.type == "int") {
            typeBytes = sizeof(int);
        } else {
            auto msg = "Unrecognized type: " + header.type;
            throw IOException(msg);
        }

        // Read data
        T t;
        std::size_t nbytes = header.width * header.dim * typeBytes;
        std::vector<T> points(header.width, 0);
        points.reserve(header.width);
        for (std::size_t h = 0; h < header.height; ++h) {
            infile.read(reinterpret_cast<char*>(points.data()), nbytes);
            ps.pushRow(points);
        }

        return ps;
    }
    /**@}*/

    /**@{*/
    /** @brief Write an ASCII PointSet */
    static void WritePointSetAscii(
        const volcart::filesystem::path& path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string()};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::MakeHeader(ps);
        outfile << header;
        for (const auto& p : ps) {
            for (std::size_t i = 0; i < T::channels; ++i) {
                outfile << p(i) << " ";
            }
            outfile << std::endl;
        }

        outfile.flush();
        outfile.close();
        if (outfile.fail()) {
            auto msg = "failure writing file '" + path.string() + "'";
            throw IOException(msg);
        }
    }

    /** @brief Write a binary PointSet */
    static void WritePointSetBinary(
        const volcart::filesystem::path& path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string(), std::ios::binary};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::MakeHeader(ps);
        outfile.write(header.c_str(), header.size());

        for (const auto& p : ps) {
            auto nbytes = T::channels * sizeof(typename T::value_type);
            outfile.write(reinterpret_cast<const char*>(p.val), nbytes);
        }

        outfile.flush();
        outfile.close();
        if (outfile.fail()) {
            auto msg = "failure writing file '" + path.string() + "'";
            throw IOException(msg);
        }
    }

    /** @brief Write an ASCII OrderedPointSet */
    static void WriteOrderedPointSetAscii(
        const volcart::filesystem::path& path, OrderedPointSet<T> ps)
    {
        std::ofstream outfile{path.string()};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::MakeOrderedHeader(ps);
        outfile << header;
        for (const auto& p : ps) {
            for (std::size_t i = 0; i < T::channels; ++i) {
                outfile << p(i) << " ";
            }
            outfile << std::endl;
        }

        outfile.flush();
        outfile.close();
        if (outfile.fail()) {
            auto msg = "failure writing file '" + path.string() + "'";
            throw IOException(msg);
        }
    }

    /** @brief Write a binary OrderedPointSet */
    static void WriteOrderedPointSetBinary(
        const volcart::filesystem::path& path, OrderedPointSet<T> ps)
    {
        std::ofstream outfile{path.string(), std::ios::binary};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::MakeOrderedHeader(ps);
        outfile.write(header.c_str(), header.size());

        for (const auto& p : ps) {
            auto nbytes = T::channels * sizeof(typename T::value_type);
            outfile.write(reinterpret_cast<const char*>(p.val), nbytes);
        }

        outfile.flush();
        outfile.close();
        if (outfile.fail()) {
            auto msg = "failure writing file '" + path.string() + "'";
            throw IOException(msg);
        }
    }
    /**@}*/
};
}  // namespace volcart
