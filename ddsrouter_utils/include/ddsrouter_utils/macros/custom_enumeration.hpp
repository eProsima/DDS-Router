// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file macros.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTERUTILS_MACROS_CUSTOMENUMERATION_HPP_
#define _DDSROUTERUTILS_MACROS_CUSTOMENUMERATION_HPP_

#include <string>
#include <array>

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/macros/macros.hpp>
#include <ddsrouter_utils/macros/recursive_macros.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief This macro creates a Custom Enumeration with auxiliary functions and variables.
 *
 * An enumeration built with ENUEMERATION_BUILDER has:
 * - enum class with name \c enumeration_name and N values, one for each extra argument, and with that exact name.
 * - array called \c nammes_<enumeration_name> with the names of each element of the enumeration
 * as strings of the enum value.
 * - \c to_string method to get the string associated with an enumeration value.
 * - \c from_string_<enumeration_name> method that gives enumeration value from string name.
 * - operator << for each enumeration value using to_string .
 * - \c N_VALUES_<enumeration_name> unsigned int to get the number of elements in the enumeration.
 *
 * @arg enumeration_name it sets the enum class name and is used to name variables and methods.
 * @arg extra_arguments each of the elements of the enum class. Their conversion to string would use this same name.
 *
 * @note empty custom enumerations are not allowed, even when empty enum class are.
 *
 * @example
 * ENUMERATION_BUILDER(CustomEnum, el1, el2);
 * CustomEnum my_value = CustomEnum::el1;  // Set my_value as el1 = 0
 * my_value = from_string_CustomEnum("el2");  // Set my_value as el2 = 1
 * to_string(my_value);  // = "el2"
 */
#define ENUMERATION_BUILDER(enumeration_name, ...)                                                                    \
                                                                                                                      \
    /* Forbid empty enumerations */                                                                                   \
    static_assert( COUNT_ARGUMENTS(__VA_ARGS__), "Empty Enumerations are not allowed.");                             \
                                                                                                                      \
    /* Declare enumeration */                                                                                         \
    enum class enumeration_name {__VA_ARGS__ \
    };                                                                        \
                                                                                                                      \
    /* Initialize name arrays */                                                                                      \
    const std::array<std::string, COUNT_ARGUMENTS(__VA_ARGS__)> names_ ## enumeration_name =                            \
    { APPLY_MACRO_FOR_EACH(STRINGIFY_WITH_COMMA, __VA_ARGS__) };                                                  \
                                                                                                                      \
    /* To string method */                                                                                            \
    const std::string& to_string(const enumeration_name& e)                                                           \
    { return names_ ## enumeration_name[static_cast<int>(e)]; }                                                     \
                                                                                                                      \
    /* From string */                                                                                                 \
    enumeration_name from_string_ ## enumeration_name(const std::string& s)                                             \
    {                                                                                                                 \
        for (int i = 0; i < COUNT_ARGUMENTS(__VA_ARGS__); i++)                                                            \
        if (names_ ## enumeration_name[i] == s)return static_cast<enumeration_name>(i);                            \
        throw eprosima::ddsrouter::utils::InitializationException(                                                    \
                  STR_ENTRY << "Not correct name " << s << " for Enum " << STRINGIFY(enumeration_name) << ".");             \
    }                                                                                                                 \
                                                                                                                      \
    /* Serialization operation */                                                                                     \
    std::ostream& operator <<(std::ostream& os, const enumeration_name& e) { os << to_string(e); return os; }         \
                                                                                                                      \
    /* Number of elements in enumeration */                                                                           \
    constexpr const unsigned int N_VALUES_ ## enumeration_name = COUNT_ARGUMENTS(__VA_ARGS__);


} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_MACROS_CUSTOMENUMERATION_HPP_ */
