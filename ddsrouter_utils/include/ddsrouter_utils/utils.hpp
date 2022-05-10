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
 * This file contains util functions for the whole project
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

using FileAccessModeType = int;

/**
   Enum for all possible access modes
 * Linux: See https://linux.die.net/man/2/access
 * Windows: See https://docs.microsoft.com/es-es/cpp/c-runtime-library/reference/access-waccess?view=msvc-170
 */
enum class FileAccessMode : FileAccessModeType
{
    exist               = 0,
    read                = 4,
    write               = 2,
    exec                = 1,
    read_write          = read | write,
    read_exec           = read | exec,
    read_write_exec     = read | write | exec,
    write_exec          = write | exec,
};

//! Overloaded '|' operator for composing permissions.
FileAccessMode operator |(
        FileAccessMode mode_a,
        FileAccessMode mode_b);

//! Overloaded '&' operator for matching permissions.
FileAccessMode operator &(
        FileAccessMode mode_a,
        FileAccessMode mode_b);

/**
 * @brief Perform the wildcard matching using file comparison method
 *
 * Wildcard matching uses * for any string of chars, ? for any char, and [a-z] for any char in the range.
 *
 * @param pattern pattern that must be matched
 * @param str string to check if matches the pattern
 * @return \c true if \c str matches the \c pattern, \c false otherwise
 */
DDSROUTER_UTILS_DllAPI bool match_pattern(
        const std::string& pattern,
        const std::string& str) noexcept;

/**
 * @brief Convert every alphabetic char in string to lower case
 *
 * @attention This function modifies the object given
 *
 * @param [in,out] st : string to modify
 */
DDSROUTER_UTILS_DllAPI void to_lowercase(
        std::string& st) noexcept;

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
DDSROUTER_UTILS_DllAPI void tsnh(
        const Formatter& formatter);

/**
 * @brief Whether a file exist and/or is accessible with specific permissions
 *
 * The accessibility could be checked for different permissions regarding argument \c access_mode ,
 * and each of them are asked by a different \c FileAccessMode :
 * - \c FileAccessMode::exist       : check if the file exist (any argument check this)
 * - \c FileAccessMode::read        : check if the file has read access
 * - \c FileAccessMode::write       : check if the file has write access
 * - \c FileAccessMode::exec        : check if the file has execution access
 *
 * This method could be used by OR \c FileAccessMode to check that file has all permissions in OR as:
 * access_mode = FileAccessMode::read_write -> check that file has read and write access permission.
 *
 * @warning windows does not retrieve information about the execution permission on a file.
 *
 * @param file_path path to the file to check
 * @param access_mode access permission(s) to check in the file
 * @return true if file is accessible regarding the permissions given
 * @return false otherwise
 */
DDSROUTER_UTILS_DllAPI bool is_file_accessible(
        const char* file_path,
        const FileAccessMode& access_mode) noexcept;

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
template <typename T, bool Ptr = false>
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
template <typename T, bool Ptr = false>
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
 * @param list set of elements
 * @param separator char or string separator between elements
 *
 * @return stream object with the concatenation of \c os and \c element
 */
template <typename T, bool Ptr = false>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::set<T>& list,
        const std::string& separator = ";");

/**
 * @brief Check if an element contained in a shared ptr is contained in a set of shared pointers
 *
 * This function is needed to find elements inside sets of shared pointers, because the operator==
 * of shared ptr is limited to check the address, and not the internal value.
 * It uses the internal operator== of \c T instead of comparing by address.
 *
 * @tparam T type of the element inside the pointer and the set
 *
 * @param set set of elements where to look for \c element
 * @param element element to look for in \c set
 *
 * @return \c true if element is inside the set, \c false otherwise
 */
template <typename T>
bool set_of_ptr_contains(
        const std::set<std::shared_ptr<T>>& set,
        const std::shared_ptr<T>& element);

//! Whether two set of shared ptrs have the same internal values (compared by object and not by address)
template <typename T>
bool are_set_of_ptr_equal(
        const std::set<std::shared_ptr<T>>& set1,
        const std::set<std::shared_ptr<T>>& set2);

/**
 * @brief Convert a elements set into a shared ptr elements set.
 */
template <typename Parent, typename Child>
std::set<std::shared_ptr<Parent>> convert_set_to_shared(
        const std::set<Child>& set);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/impl/utils.ipp>

#endif /* _DDSROUTERUTILS_UTILS_HPP_ */
