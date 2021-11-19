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
 * @brief Auxiliar function to concatenate inplace every kind of object << stream
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

    //! Concatenated stream where the stremas are added at the end
    std::stringstream ss_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_UTILS_HPP_ */
