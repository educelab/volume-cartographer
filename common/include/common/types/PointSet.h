#pragma once

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace volcart
{

template <typename T>
class PointSet
{
public:
    using Element = T;
    using Container = std::vector<Element>;
    using Iterator = typename Container::iterator;
    using ConstIterator = typename Container::const_iterator;
    constexpr static int FORMAT_VERSION = 1;
    constexpr static auto HEADER_TERMINATOR = "<>";

    // Header definition
    struct Header {
        size_t width;
        size_t height;
        size_t size;
        size_t dim;
        bool ordered;
        std::string type;

        Header() : width(0), height(0), size(0), dim(0), ordered(false), type()
        {
        }
    };

    PointSet() : _width(0), _height(0), _nelements(0), _ordered(false), _data()
    {
    }

    PointSet(size_t size)
        : _width(0), _height(0), _nelements(size), _ordered(false)
    {
        _data.reserve(_nelements);
    }

    PointSet(size_t width, size_t height)
        : _width(width)
        , _height(height)
        , _nelements(width * height)
        , _ordered(true)
    {
        _data.reserve(_nelements);
    }

    PointSet(size_t width, size_t height, T init_val)
        : _width(width)
        , _height(height)
        , _nelements(width * height)
        , _ordered(true)
    {
        _data.assign(_nelements, init_val);
    }

    // Linear access - no concept of 2D layout
    const T& operator[](size_t idx) const { return _data[idx]; }
    T& operator[](size_t idx) { return _data[idx]; }

    // 2D access
    // NOTE: x, then y
    const T& operator()(size_t x, size_t y) const
    {
        assert(_ordered && "Cannot use operator() with unordered PointSet");
        return _data[y * _width + x];
    }
    T& operator()(size_t x, size_t y)
    {
        assert(_ordered && "Cannot use operator() with unordered PointSet");
        return _data[y * _width + x];
    }

    // Metadata
    size_t width() const { return _width; }
    void setWidth(size_t width) { _width = width; }
    size_t height() const { return _height; }
    void setHeight(size_t height) { _height = height; }
    size_t size() const { return _nelements; }
    bool empty() const { return _data.empty(); }
    bool isOrdered() const { return _ordered; }
    void setOrdered(size_t newWidth, size_t newHeight)
    {
        _ordered = true;
        _width = newWidth;
        _height = newHeight;
    }
    void setUnordered()
    {
        _ordered = false;
        _width = _height = 0;
    }

    // Iterators and element accessors
    Iterator begin() { return std::begin(_data); }
    ConstIterator begin() const { return std::begin(_data); }
    Iterator end() { return std::end(_data); }
    ConstIterator end() const { return std::end(_data); }
    T& front() { return _data.front(); }
    const T& front() const { return _data.front(); }
    T& back() { return _data.back(); }
    const T& back() const { return _data.back(); }

    // Some basic statistics
    T min() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::min_element(std::begin(_data), std::end(_data));
    }
    T max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::max_element(std::begin(_data), std::end(_data));
    }
    std::pair<T, T> min_max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        auto pair = std::minmax_element(std::begin(_data), std::end(_data));
        return {*pair.first, *pair.second};
    }

    // I/O modes
    enum class IOMode { ASCII, BINARY };

    static PointSet<T> readFile(boost::filesystem::path path,
                                IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
        case IOMode::BINARY:
            return readFileBinary(path);
        case IOMode::ASCII:
            return readFileAscii(path);
        }
    }

    static void writeFile(boost::filesystem::path path,
                          const PointSet<T>& ps,
                          IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
        case IOMode::BINARY:
            return writeFileBinary(path, ps);
        case IOMode::ASCII:
            return writeFileAscii(path, ps);
        }
    }

    void push_back(const T& val) { _data.push_back(val); }

    void push_back(T&& val) { _data.push_back(val); }

    void push_row(const std::vector<T>& points)
    {
        assert(points.size() == _width && "row incorrect size");
        std::copy(std::begin(points), std::end(points),
                  std::back_inserter(_data));
    }

    void push_row(std::vector<T>&& points)
    {
        assert(points.size() == _width && "row incorrect size");
        std::copy(std::begin(points), std::end(points),
                  std::back_inserter(_data));
    }

private:
    size_t _width;
    size_t _height;
    size_t _nelements;
    bool _ordered;
    Container _data;

    static PointSet<T> readFileAscii(boost::filesystem::path path)
    {
        std::ifstream infile{path.string()};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw std::runtime_error(msg);
        }

        // Get header
        auto header = PointSet<T>::parseHeader(infile);

        // Get data
        PointSet<T> ps;
        if (header.ordered) {
            ps.setOrdered(header.width, header.height);
        }
        T tmp;
        for (size_t i = 0; i < ps.size(); ++i) {
            infile >> tmp;
            ps.push_back(tmp);
        }

        return ps;
    }

    static PointSet<T> readFileBinary(boost::filesystem::path path)
    {
        std::ifstream infile{path.string(), std::ios::binary};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw std::runtime_error(msg);
        }
        auto header = PointSet<T>::parseHeader(infile);
        infile.get();

        size_t typeBytes;
        if (header.type == "float") {
            typeBytes = sizeof(float);
        } else if (header.type == "double") {
            typeBytes = sizeof(double);
        } else if (header.type == "int") {
            typeBytes = sizeof(int);
        }

        PointSet<T> ps;
        if (header.ordered) {
            ps.setOrdered(header.width, header.height);
        }
        T t;
        for (size_t i = 0; i < ps.size(); ++i) {
            auto nbytes = header.dim * typeBytes;
            infile.read(t.bytes(), nbytes);
            ps.push_back(t);
        }

        return ps;
    }

    static void writeFileAscii(boost::filesystem::path path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string()};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw std::runtime_error(msg);
        }

        auto header = PointSet<T>::makeHeader(ps);
        outfile << header;
        for (const auto& p : ps) {
            outfile << p << std::endl;
        }
    }

    static void writeFileBinary(boost::filesystem::path path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string(), std::ios::binary};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw std::runtime_error(msg);
        }

        auto header = PointSet<T>::makeHeader(ps);
        outfile.write(header.c_str(), header.size());

        for (const auto p : ps) {
            auto nbytes = decltype(p)::dim * sizeof(typename T::Element);
            outfile.write(p.bytes(), nbytes);
        }
    }

    static std::string makeHeader(PointSet<T> ps)
    {
        std::stringstream ss;
        if (ps.isOrdered()) {
            ss << "height: " << ps.height() << std::endl;
            ss << "width: " << ps.height() << std::endl;
        } else {
            ss << "size: " << ps.size() << std::endl;
        }
        ss << "dim: " << PointSet<T>::Element::dim << std::endl;
        ss << "ordered: " << (ps.isOrdered() ? "true" : "false") << std::endl;

        // Output type information
        ss << "type: ";
        if (std::is_same<typename T::Element, int>::value) {
            ss << "int" << std::endl;
        } else if (std::is_same<typename T::Element, float>::value) {
            ss << "float" << std::endl;
        } else if (std::is_same<typename T::Element, double>::value) {
            ss << "double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw std::runtime_error(msg);
        }

        ss << "version: " << PointSet<T>::FORMAT_VERSION << std::endl;
        ss << PointSet<T>::HEADER_TERMINATOR << std::endl;

        return ss.str();
    }

    // Note: assumes infile is already open
    static Header parseHeader(std::ifstream& infile)
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
        std::regex validTypes{"(float)|(double)|(int)"};
        std::regex headerTerminator{HEADER_TERMINATOR};

        Header h;
        std::string line;
        std::vector<std::string> strs;

        while (std::getline(infile, line) &&
               !std::regex_match(line, headerTerminator)) {
            boost::trim(line);
            std::cout << "line: " << line << std::endl;

            // Comments: look like:
            // # This is a comment
            //    # This is another comment
            if (std::regex_match(line, comments)) {
                continue;
            }

            // Width
            else if (std::regex_match(line, width)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                h.width = std::stoul(strs[1]);
            }

            // Height
            else if (std::regex_match(line, height)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                h.height = std::stoul(strs[1]);
            }

            // Size
            else if (std::regex_match(line, size)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                h.height = std::stoul(strs[1]);
            }

            // Dim
            else if (std::regex_match(line, dim)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                auto dim = std::stoul(strs[1]);
                if (dim != T::dim) {
                    auto msg =
                        "Incorrect dimension read for template specification";
                    throw std::runtime_error(msg);
                }
                h.dim = dim;
            }

            // Ordering
            else if (std::regex_match(line, ordering)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                boost::algorithm::to_lower(strs[1]);
                if (strs[1] == "true") {
                    h.ordered = true;
                } else if (strs[1] == "false") {
                    h.ordered = false;
                } else {
                    auto msg = "'ordered' key must have value 'true'/'false'";
                    throw std::runtime_error(msg);
                }
            }

            // Type
            else if (std::regex_match(line, type)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                if (!std::regex_match(strs[1], validTypes)) {
                    auto msg = "Valid types are int, float, double. Got: '" +
                               strs[1] + "'";
                    throw std::runtime_error(msg);
                }

                // Type validation
                auto msg = "Type mismatch: vcps filetype '" + strs[1] +
                           "' not compatible with reader type 'int'";
                if (strs[1] == "int" &&
                    !std::is_same<typename T::Element, int>::value) {
                    throw std::runtime_error(msg);
                } else if (strs[1] == "float" &&
                           !std::is_same<typename T::Element, float>::value) {
                    throw std::runtime_error(msg);
                } else if (strs[1] == "double" &&
                           !std::is_same<typename T::Element, double>::value) {
                    throw std::runtime_error(msg);
                }
                std::cout << "read type: " << strs[1] << std::endl;
                h.type = strs[1];
            }

            // Version
            else if (std::regex_match(line, version)) {
                boost::split(strs, line, boost::is_any_of(":"));
                boost::trim(strs[1]);
                auto fileVersion = std::stoi(strs[1]);
                if (fileVersion != FORMAT_VERSION) {
                    auto msg = "Version mismatch. VCPS file version is " +
                               strs[1] + ", processing version is " +
                               std::to_string(FORMAT_VERSION) + ".";
                    throw std::runtime_error(msg);
                }
            }

            // Ignore everything else
            else {
                continue;
            }
            strs.clear();
        }

        // Sanity check. Do we have a valid pointset header?
        if (h.type == "") {
            auto msg = "Must provide type";
            throw std::runtime_error(msg);
        } else if (h.dim == 0) {
            auto msg = "Msut provide dim";
            throw std::runtime_error(msg);
        } else if (h.ordered == false && h.size == 0) {
            auto msg = "Unordered pointsets must have a size";
            throw std::runtime_error(msg);
        } else if (h.ordered == true && (h.width == 0 || h.height == 0)) {
            auto msg = "Ordered pointsets must have a nonzero width and height";
            throw std::runtime_error(msg);
        }

        return h;
    }
};
}
