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

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ddsrouter/types/macros.hpp>

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

    for (size_t i = 0; size != 0 && i < size - 1; ++i)
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

template <typename T>
bool set_of_ptr_contains(
        const std::set<std::shared_ptr<T>> set,
        const std::shared_ptr<T> element)
{
    // If the pointer belongs to set, it is contained
    if (set.find(element) != set.end())
    {
        return true;
    }

    // If element we are looking for is nullptr, do not search as we already know it is not in the set
    // (every nullptr is compared to equal)
    if (!element)
    {
        return false;
    }

    // If not, check if any object internally is the one we are looking for
    for (auto itr = set.begin(); itr != set.end(); itr++)
    {
        // In case the set element is nullptr, do not call == or it will crash
        if (nullptr != *itr)
        {
            if (*itr->get() == *element.get())
            {
                // If are equal, element is contained
                return true;
            }
        }
    }

    return false;
}

template <typename T>
bool are_set_of_ptr_equal(
        const std::set<std::shared_ptr<T>> set1,
        const std::set<std::shared_ptr<T>> set2)
{
    if (set1.size() != set2.size())
    {
        return false;
    }

    // Check if every element in set1 is in set2
    for (auto itr1 = set1.begin(); itr1 != set1.end(); itr1++)
    {
        if (!set_of_ptr_contains(set2, *itr1))
        {
            return false;
        }
    }

    return true;
}

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
void tsnh(
        const Formatter& formatter);

/**
 * @brief Convert a elements set into a shared ptr elements set.
 */
template <typename Parent, typename Child>
std::set<std::shared_ptr<Parent>> convert_set_to_shared(std::set<Child> set)
{
    FORCE_TEMPLATE_SUBCLASS(Parent, Child);

    std::set<std::shared_ptr<Parent>> result_set;
    for (Child element : set)
    {
        result_set.insert(std::make_shared<Child>(element));
    }
    return result_set;
}

// TODO : move templates to ipp file

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_UTILS_HPP_ */
