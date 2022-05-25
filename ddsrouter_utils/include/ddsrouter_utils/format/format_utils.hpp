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
 * @file format_utils.hpp
 *
 * This file contains util functions to format different elements into strings or streams
 */

#ifndef _DDSROUTERUTILS_FORMAT_FORMATUTILS_HPP_
#define _DDSROUTERUTILS_FORMAT_FORMATUTILS_HPP_

#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Convert every alphabetic char in string to lower case
 *
 * @attention This function modifies the object given
 *
 * @param [in,out] st : string to modify
 */
DDSROUTER_UTILS_DllAPI void to_lowercase(
        std::string& st) noexcept;

/////
// TEMPLATE FUNCTIONS

/**
 * @brief Concatenate an element \c element into a stream by operator <<
 *
 * @tparam T type of each element. This object must have an << operator
 * @tparam Ptr whether \c T is a pointer. In case it is true, it is used the internal element and not the address
 *
 * @param os stream to store the concatenation result
 * @param element element to concatenate into stream
 *
 * @return stream object with the concatenation of \c os and \c element
 */
template <typename T>
std::ostream& element_to_stream(
        std::ostream& os,
        const T& element);

/**
 * @brief Concatenate serialization of elements in a vector separated by \c separator .
 *
 * @tparam T type of each element. This object must have an << operator
 * @tparam Ptr whether \c T is a pointer. In case it is true, it is used the internal element and not the address
 *
 * @param os stream to store the concatenation result
 * @param list vector of elements
 * @param separator char or string separator between elements
 *
 * @return stream object with the concatenation of \c os and \c element
 */
template <typename T>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::vector<T>& list,
        const std::string& separator = ";");

/**
 * @brief Concatenate serialization of elements in a set separated by \c separator .
 *
 * The order of the elements in the final stream is not guaranteed.
 *
 * @tparam T type of each element. This object must have an << operator
 * @tparam Ptr whether \c T is a pointer. In case it is true, it is used the internal element and not the address
 *
 * @param os stream to store the concatenation result
 * @param set set of elements
 * @param separator char or string separator between elements
 *
 * @return stream object with the concatenation of \c os and \c element
 */
template <typename T>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::set<T>& set,
        const std::string& separator = ";");

template <typename T>
DDSROUTER_UTILS_DllAPI std::string to_string(
        const T& element) noexcept;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/format/impl/format_utils.ipp>

#endif /* _DDSROUTERUTILS_FORMAT_FORMATUTILS_HPP_ */
