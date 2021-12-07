// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file utils.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTER_TYPES_UTILS_HPP_
#define _DDSROUTER_TYPES_UTILS_HPP_

#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Perform the wildcard matching using file comparison method
bool match_pattern(
        const std::string& pattern,
        const std::string& str) noexcept;

/**
 * @brief Convert every alphabetic char in string to lower case
 *
 * @attention This function modifies the object given
 *
 * @param [in,out] st : string to modify
 */
void to_lowercase(
        std::string& st) noexcept;

/**
 * @brief Auxiliary function to concatenate inplace every kind of object << stream
 *
 * The main case to use this class is in Exception creation. In order to generate an Exception message
 * using the << operator for the objects in the block, add them to Formatter() object and they will be
 * concatenated in a single string. For example:
 * Exception(Formatter() << " object1 stream: " << obj1 << " object2 stream: " << obj2);
 */
class Formatter
{
public:

    //! Concatenate stream values to this formatter
    template<class Val> Formatter& operator <<(
            const Val& val)
    {
        ss_ << val;
        return *this;
    }

    //! Return a string with the concatenation of this object
    std::string to_string() const noexcept;

protected:

    //! Concatenated stream where the streams are added at the end
    std::stringstream ss_;
};

template <typename T, bool Ptr = false>
std::ostream& element_to_stream(
    std::ostream& os,
    T element)
{
    if (Ptr)
    {
        os << (*element);
    }
    else
    {
        os << element;
    }

    return os;
}

/**
 * @brief Concatenate serialization of elements in an array separated by \c separator .
 *
 * @tparam T type of each element. This object must have an << operator
 * @param os stream to store the concatenation result
 * @param list list of elements
 * @param separator char or string separator between elements
 * @return std::ostream& with the result stream concatenated
 */
template <typename T, bool Ptr = false>
std::ostream& container_to_stream(
    std::ostream& os,
    std::vector<T> list,
    std::string separator = ";")
{
    os << "{";

    size_t size = list.size();

    for (size_t i=0; size!=0 && i<size-1; ++i)
    {
        element_to_stream<T, Ptr>(os, list[i]);
        os << separator;
    }

    if (size > 0)
    {
        element_to_stream<T, Ptr>(os, list[size - 1]);
    }

    os << "}";

    return os;
}

//! Concatenate a set by converting to vector.
template <typename T, bool Ptr = false>
std::ostream& container_to_stream(
    std::ostream& os,
    std::set<T> list,
    std::string separator = ";")
{
    return container_to_stream<T, Ptr>(os, std::vector<T>(list.begin(), list.end()), separator);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_UTILS_HPP_ */
