#pragma once

#include <boost/filesystem/path.hpp>
#include <fstream>
#include <iostream>
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

    PointSet()
        : _width(0), _height(0), _nelements(0), _isOrdered(false), _data()
    {
    }

    PointSet(size_t size)
        : _width(0), _height(0), _nelements(size), _isOrdered(false)
    {
        _data.reserve(_nelements);
    }

    PointSet(size_t width, size_t height)
        : _width(width)
        , _height(height)
        , _nelements(width * height)
        , _isOrdered(true)
    {
        _data.reserve(_nelements);
    }

    PointSet(size_t width, size_t height, T init_val)
        : _width(width)
        , _height(height)
        , _nelements(width * height)
        , _isOrdered(true)
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
        assert(_isOrdered && "Cannot use operator() with unordered pointset");
        return _data[y * _width + x];
    }
    T& operator()(size_t x, size_t y)
    {
        assert(_isOrdered && "Cannot use operator() with unordered pointset");
        return _data[y * _width + x];
    }

    // Metadata
    size_t width() const { return _width; }
    void setWidth(size_t width) { _width = width; }
    size_t height() const { return _height; }
    void setHeight(size_t height) { _height = height; }
    size_t size() const { return _nelements; }
    bool empty() const { return _data.empty(); }
    bool isOrdered() const { return _isOrdered; }
    void setOrdered(size_t newWidth, size_t newHeight)
    {
        _isOrdered = true;
        _width = newWidth;
        _height = newHeight;
    }
    void setUnordered()
    {
        _isOrdered = false;
        _width = _height = 0;
    }

    // Iterators and element accesors
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
            throw std::range_error("empty volcart::PointSet");
        }
        return *std::min_element(std::begin(_data), std::end(_data));
    }
    T max() const
    {
        if (empty()) {
            throw std::range_error("empty volcart::PointSet");
        }
        return *std::max_element(std::begin(_data), std::end(_data));
    }
    std::pair<T, T> min_max() const
    {
        if (empty()) {
            throw std::range_error("empty volcart::PointSet");
        }
        auto pair = std::minmax_element(std::begin(_data), std::end(_data));
        return {*pair.first, *pair.second};
    }

    // I/O modes
    enum class IOMode { ASCII, BINARY };

    /*
     * Assumes file is of the form:
     *     width <width>
     *     height <height>
     *     type <type>
     *     dim <dim>
     *     <data>
     */
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

private:
    size_t _width;
    size_t _height;
    size_t _nelements;
    bool _isOrdered;
    Container _data;

    static PointSet<T> readFileAscii(boost::filesystem::path path)
    {
        std::ifstream infile{path.string()};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw std::runtime_error(msg);
        }

        // Get width, height
        std::string key;
        size_t width, height, dim, i = 0;
        std::string type;
        while (i < 4) {
            infile >> key;
            if (key.empty()) {
                continue;
            } else if (key == "width") {
                infile >> width;
                ++i;
            } else if (key == "height") {
                infile >> height;
                ++i;
            } else if (key == "dim") {
                infile >> dim;
                ++i;
            } else if (key == "type") {
                infile >> type;
                ++i;
            } else {
                std::string msg = "Got unknown key '" + key + "'";
                throw std::runtime_error(msg);
            }
        }

        // Basic shitty stringly-typed checking
        assert((type == "double" || type == "float" || type == "int") &&
               "Only supports types 'float', 'int', and 'double'");
        assert(dim == T::dim && "Wrong dimension read");

        // Get data
        PointSet<T> ps(width, height);
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

        // Get width, height, dim, type
        std::string key;
        size_t width, height, dim, i = 0;
        std::string type;
        while (i < 4) {
            infile >> key;
            if (key.empty()) {
                continue;
            } else if (key == "width") {
                infile >> width;
                ++i;
            } else if (key == "height") {
                infile >> height;
                ++i;
            } else if (key == "dim") {
                infile >> dim;
                ++i;
            } else if (key == "type") {
                infile >> type;
                ++i;
            } else {
                std::string msg = "Got unknown key '" + key + "'";
                throw std::runtime_error(msg);
            }
        }
        infile.get();

        // Basic shitty stringly-typed checking
        assert((type == "double" || type == "float" || type == "int") &&
               "Only supports types 'float', 'int', and 'double'");
        assert(dim == T::dim && "Wrong dimension read");

        size_t typeBytes;
        if (type == "float") {
            typeBytes = sizeof(float);
        } else if (type == "double") {
            typeBytes = sizeof(double);
        } else if (type == "int") {
            typeBytes = sizeof(int);
        }

        PointSet<T> ps(width, height);
        T t;
        for (size_t i = 0; i < ps.size(); ++i) {
            auto nbytes = dim * typeBytes;
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

        outfile << "width " << ps.width() << std::endl;
        outfile << "height " << ps.height() << std::endl;
        outfile << "dim " << decltype(ps)::Element::dim << std::endl;

        // Output type information
        if (std::is_same<typename T::Element, int>::value) {
            outfile << "type int" << std::endl;
        } else if (std::is_same<typename T::Element, float>::value) {
            outfile << "type float" << std::endl;
        } else if (std::is_same<typename T::Element, double>::value) {
            outfile << "type double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw std::runtime_error(msg);
        }

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

        std::stringstream ss;
        ss << "width " << ps.width() << std::endl;
        ss << "height " << ps.height() << std::endl;
        ss << "dim " << decltype(ps)::Element::dim << std::endl;

        // Output type information
        if (std::is_same<typename T::Element, int>::value) {
            ss << "type int" << std::endl;
        } else if (std::is_same<typename T::Element, float>::value) {
            ss << "type float" << std::endl;
        } else if (std::is_same<typename T::Element, double>::value) {
            ss << "type double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw std::runtime_error(msg);
        }

        outfile.write(ss.str().c_str(), ss.str().size());

        for (const auto p : ps) {
            auto nbytes = decltype(p)::dim * sizeof(typename T::Element);
            outfile.write(p.bytes(), nbytes);
        }
    }
};
}
