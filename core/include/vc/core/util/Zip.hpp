/**
 * @file Zip.hpp
 * @brief Make a variable-sized tuple
 *
 * @ingroup Util
 */
#pragma once

#include <boost/iterator/zip_iterator.hpp>
#include <boost/range.hpp>
#include <boost/range/iterator_range.hpp>

namespace volcart
{
/**
 * @brief Make a variable-sized tuple
 */
template <typename... T>
auto zip(const T&... cs) -> boost::iterator_range<
    boost::zip_iterator<decltype(boost::make_tuple(std::begin(cs)...))>>
{
    using boost::make_zip_iterator;
    using boost::make_tuple;
    auto zip_begin = make_zip_iterator(make_tuple(std::begin(cs)...));
    auto zip_end = make_zip_iterator(make_tuple(std::end(cs)...));
    return boost::make_iterator_range(zip_begin, zip_end);
}
}
