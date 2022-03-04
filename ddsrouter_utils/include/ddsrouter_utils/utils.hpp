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

#ifndef _DDSROUTERUTILS_UTILS_HPP_
#define _DDSROUTERUTILS_UTILS_HPP_

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ddsrouter_utils/macros.hpp>
#include <ddsrouter_utils/Formatter.hpp>
#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Perform the wildcard matching using file comparison method
DDSROUTERUTILS_DllAPI bool match_pattern(
        const std::string& pattern,
        const std::string& str) noexcept;

/**
 * @brief Convert every alphabetic char in string to lower case
 *
 * @attention This function modifies the object given
 *
 * @param [in,out] st : string to modify
 */
DDSROUTERUTILS_DllAPI void to_lowercase(
        std::string& st) noexcept;

template <typename T, bool Ptr = false>
DDSROUTERUTILS_DllAPI std::ostream& element_to_stream(
        std::ostream& os,
        T element);

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
DDSROUTERUTILS_DllAPI std::ostream& container_to_stream(
        std::ostream& os,
        std::vector<T> list,
        std::string separator = ";");

//! Concatenate a set by converting to vector.
template <typename T, bool Ptr = false>
DDSROUTERUTILS_DllAPI std::ostream& container_to_stream(
        std::ostream& os,
        std::set<T> list,
        std::string separator = ";");

template <typename T>
DDSROUTERUTILS_DllAPI bool set_of_ptr_contains(
        const std::set<std::shared_ptr<T>> set,
        const std::shared_ptr<T> element);

template <typename T>
DDSROUTERUTILS_DllAPI bool are_set_of_ptr_equal(
        const std::set<std::shared_ptr<T>> set1,
        const std::set<std::shared_ptr<T>> set2);

/**
 * @brief This Should Not Happen.
 *
 * This method should be called when something that should not have happened at all happens.
 * This will show an error log, assert false and throw an exception.
 *
 * Do not use this method when the error could come from user or output interaction, it should only be used
 * for inconsistency inside the program or C++ weird behaviours (e.g. enumeration values out of their range).
 *
 * @param formatter msg of the unexpected case.
 */
DDSROUTERUTILS_DllAPI void tsnh(
        const Formatter& formatter);

/**
 * @brief Convert a elements set into a shared ptr elements set.
 */
template <typename Parent, typename Child>
DDSROUTERUTILS_DllAPI std::set<std::shared_ptr<Parent>> convert_set_to_shared(
        std::set<Child> set);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/impl/utils.ipp>

#endif /* _DDSROUTERUTILS_UTILS_HPP_ */
