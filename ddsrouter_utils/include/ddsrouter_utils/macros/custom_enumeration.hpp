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
 * @brief This macro creates a Custom Enumeration
 *
 * A Custom Enumeration is an Enumeration class that internally has a enum class and has several methods to interact
 * with it:
 * - creation from enum value
 * - creation from string
 * - conversion to string
 * - conversion to int
 *
 * @warning Requires a ";" after call
 */

// #define ENUMERATION_BUILDER(enumeration_name, ...) \
//                                                                                                                        \
//     class enumeration_name                                                                                             \
//     {                                                                                                                  \
//     public:                                                                                                            \
//                                                                                                                        \
//         /* Internal enumeration */                                                                                     \
//         enum class Enum {__VA_ARGS__};                                                                                 \
//                                                                                                                        \
//         /* Constructor from Enumeration */                                                                             \
//         enumeration_name(Enum e) : internal_value_(e) {}                                                               \
//                                                                                                                        \
//         /* Constructor from string */                                                                                  \
//         enumeration_name(const std::string& s) : internal_value_(from_string_(s)) {}                                   \
//                                                                                                                        \
//         /* To string method */                                                                                         \
//         std::string to_string() const { return names_[static_cast<int>(internal_value_)]; }                            \
//                                                                                                                        \
//         /* To string operator */                                                                                       \
//         operator std::string() const { return this->to_string(); }                                                     \
//                                                                                                                        \
//         /* To int operator */                                                                                          \
//         operator int() const { return static_cast<int>(internal_value_); }                                             \
//                                                                                                                        \
//         /* To Enum operator */                                                                                         \
//         operator Enum() const { return internal_value_; }                                                              \
//                                                                                                                        \
//         /* Equal comparision */                                                                                        \
//         bool operator== (const enumeration_name& other) { return this->internal_value_ == other.internal_value_; }     \
//                                                                                                                        \
//         /* Equal comparision */                                                                                        \
//         bool operator== (const Enum& other) { return this->internal_value_ == other; }                                 \
//                                                                                                                        \
//         /* Not Equal comparision */                                                                                    \
//         bool operator!= (const enumeration_name& other) { return this->internal_value_ != other.internal_value_; }     \
//                                                                                                                        \
//         /* Not Equal comparision */                                                                                    \
//         bool operator!= (const Enum& other) { return this->internal_value_ != other; }                                 \
//                                                                                                                        \
//     protected:                                                                                                         \
//                                                                                                                        \
//         /* From string */                                                                                              \
//         static Enum from_string_ (const std::string& s)                                                                \
//         {                                                                                                              \
//             for (int i=0; i<COUNT_ARGUMENTS(__VA_ARGS__); i++)                                                         \
//                 if (names_[i] == s) return static_cast<Enum>(i);                                                       \
//             throw eprosima::ddsrouter::utils::InitializationException(                                                 \
//                 STR_ENTRY << "Not correct name " << s << " for Enum " << STRINGIFY(enumeration_name) << ".");          \
//         }                                                                                                              \
//                                                                                                                        \
//                                                                                                                        \
//         /* Names array */                                                                                              \
//         static const std::array<std::string, COUNT_ARGUMENTS(__VA_ARGS__)> names_;                                     \
//                                                                                                                        \
//         /* Internal value */                                                                                           \
//         Enum internal_value_;                                                                                          \
//                                                                                                                        \
//     };                                                                                                                 \
//                                                                                                                        \
//     /* Serialization operation */                                                                                      \
//     std::ostream& operator <<(std::ostream& os, const enumeration_name& e) { os << e.to_string(); return os; }         \
//                                                                                                                        \
//     /* Initialize name arrays */                                                                                       \
//     const std::array<std::string, COUNT_ARGUMENTS(__VA_ARGS__)> enumeration_name::names_ =                             \
//         { APPLY_MACRO_FOR_EACH(STRINGIFY_WITH_COMMA, __VA_ARGS__) }




#define ENUMERATION_BUILDER(enumeration_name, ...) \
                                                                                                                      \
    /* Declare enumeration */                                                                                         \
    enum class enumeration_name {__VA_ARGS__};                                                                        \
                                                                                                                      \
    /* Initialize name arrays */                                                                                      \
    const std::array<std::string, COUNT_ARGUMENTS(__VA_ARGS__)> names_##enumeration_name =                            \
        { APPLY_MACRO_FOR_EACH(STRINGIFY_WITH_COMMA, __VA_ARGS__) };                                                  \
                                                                                                                      \
    /* To string method */                                                                                            \
    const std::string& to_string(const enumeration_name& e)                                                           \
        { return names_##enumeration_name[static_cast<int>(e)]; }                                                     \
                                                                                                                      \
    /* From string */                                                                                                 \
    enumeration_name from_string_##enumeration_name(const std::string& s)                                             \
    {                                                                                                                 \
        for (int i=0; i<COUNT_ARGUMENTS(__VA_ARGS__); i++)                                                            \
            if (names_##enumeration_name[i] == s) return static_cast<enumeration_name>(i);                            \
        throw eprosima::ddsrouter::utils::InitializationException(                                                    \
            STR_ENTRY << "Not correct name " << s << " for Enum " << STRINGIFY(enumeration_name) << ".");             \
    }                                                                                                                 \
                                                                                                                      \
    /* Serialization operation */                                                                                     \
    std::ostream& operator <<(std::ostream& os, const enumeration_name& e) { os << to_string(e); return os; }


} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_MACROS_CUSTOMENUMERATION_HPP_ */
