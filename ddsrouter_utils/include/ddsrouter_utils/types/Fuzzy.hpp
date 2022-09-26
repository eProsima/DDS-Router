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
 * @file Fuzzy.hpp
 *
 * This file contains class Formatter implementation.
 */

#ifndef _DDSROUTERUTILS_TYPES_FUZZY_HPP_
#define _DDSROUTERUTILS_TYPES_FUZZY_HPP_

#include <ostream>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/macros/custom_enumeration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

using FuzzyLevelType = short;

enum FuzzyLevelValues : FuzzyLevelType
{
    fuzzy_level_unset = -20,
    fuzzy_level_default = 0,
    fuzzy_level_fuzzy = 20,
    fuzzy_level_set = 40,
    fuzzy_level_hard = 60,
};

/**
 * Fuzzy class is an implementation of a class related with a type \c T and a certainty level that this value has
 * been set.
 *
 * It is useful to maintain together the logic of a class/value and the knowledge if the element has been set by
 * default, has not been set at all, or it has been set, and with which certainty has been set.
 *
 * @example
 * void use_a_if_set(Fuzzy<A> a) {
 *   // If a has been set, use it, if not do nothing
 *   if (a.is_set()) a.get_reference().foo();  // Calling A::foo() for object A inside a
 * }
 *
 * int main() {
 *   Fuzzy<A> a;  // This is not set, calling use_if_set will do nothing
 *   Fuzzy<A> b(...);  // This has been set, calling use_if_set will call foo
 * }
 *
 *
 * @note This does not inherit from T because inheritance is not supported for native types
 */
template <typename T>
struct Fuzzy
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    Fuzzy() = default;

    Fuzzy(
        const T& other,
        FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    Fuzzy(
        T&& other,
        FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    /////////////////////////
    // OPERATORS
    /////////////////////////

    operator const T&() const noexcept;

    bool operator==(const Fuzzy<T>& other) const noexcept;

    bool operator==(const T& other) const noexcept;

    bool operator!=(const Fuzzy<T>& other) const noexcept;

    bool operator!=(const T& other) const noexcept;

    /////////////////////////
    // GET METHODS
    /////////////////////////

    bool is_valid() const noexcept;

    bool is_set() const noexcept;

    const T& get_reference() const noexcept;

    /////////////////////////
    // SET METHODS
    /////////////////////////

    void unset();

    void set_value(const T& new_value, FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    void set_level(FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    /////////////////////////
    // VARIABLES
    /////////////////////////

    FuzzyLevelType fuzzy_level = FuzzyLevelValues::fuzzy_level_default;

    T value = T();
};


/////////////////////////
// SERIALIZATION
/////////////////////////

//! \c Fuzzy to stream serializator
template <typename T>
DDSROUTER_UTILS_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Fuzzy<T>& f);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/types/impl/Fuzzy.ipp>

#endif /* _DDSROUTERUTILS_TYPES_FUZZY_HPP_ */
