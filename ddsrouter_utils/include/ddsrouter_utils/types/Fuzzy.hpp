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

//! Data Type of the value that stores the Fuzzy level
using FuzzyLevelType = short;

/**
 * @brief Guide values for the meaning of the Fuzzy level
 *
 * This values are not all the possible values of a Fuzzy level.
 * A Fuzzy level could be any number in range of \c FuzzyLevelType .
 * This values are only a guide to know what each level refers to.
 */
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
 *   Fuzzy<A> b(...);  // This has been set, calling use_if_set will call foo()
 * }
 *
 * @tparam T type of the internal element.
 *
 * @warning \c T must have a default constructor.
 *
 * @note This does not inherit from T because inheritance is not supported for native types.
 *
 * @todo Create this class as inheritance of T and create a parallel function for native types.
 */
template <typename T>
class Fuzzy
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    /**
     * @brief Default constructor of a T object without arguments.
     *
     * Fuzzy Level is set to \c fuzzy_level_default .
     */
    Fuzzy() = default;

    /**
     * @brief Construct a new Fuzzy object copying from \c other .
     *
     * @param other \c T value to copy in new object
     * @param level level of fuzzy with it is set (Default: \c fuzzy_level_set ).
     */
    Fuzzy(
            const T& other,
            FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    /**
     * @brief Construct a new Fuzzy object moving from \c other .
     *
     * @param other \c T value to move in new object
     * @param level level of fuzzy with it is set (Default: \c fuzzy_level_set ).
     */
    Fuzzy(
            T&& other,
            FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    /////////////////////////
    // OPERATORS
    /////////////////////////

    //! Access to internal value reference
    operator T&() noexcept;

    //! Implicit cast to object T
    operator T() const noexcept;

    /**
     * @brief Comparison operator with a Fuzzy object.
     *
     * @param other value to compare
     * @return true if both are invalid, or if both are set and internal value is the same
     * @return false otherwise
     */
    bool operator ==(
            const Fuzzy<T>& other) const noexcept;

    /**
     * @brief Comparison operator with a \c T object.
     *
     * @param other value to compare
     * @return true if this is valid and internal value is the same as \c other .
     * @return false otherwise
     */
    bool operator ==(
            const T& other) const noexcept;

    //! Opposite of \c operator== .
    bool operator !=(
            const Fuzzy<T>& other) const noexcept;

    //! Opposite of \c operator== .
    bool operator !=(
            const T& other) const noexcept;

    /////////////////////////
    // GET METHODS
    /////////////////////////

    //! Whether the internal Fuzzy Level is equal or higher \c fuzzy_level_default .
    bool is_valid() const noexcept;

    //! Whether the internal Fuzzy Level is equal or higher \c fuzzy_level_fuzzy .
    bool is_set() const noexcept;

    //! Get reference of the internal value
    T& get_reference() noexcept;
    const T& get_reference() const noexcept;

    //! Get internal value
    T get_value() const noexcept;

    //! Get the Fuzzy Level internal value.
    FuzzyLevelType get_level() const noexcept;

    /////////////////////////
    // SET METHODS
    /////////////////////////

    //! Set Fuzzy Level to \c fuzzy_level_unset .
    void unset();

    //! Set internal value to \c new_value and Fuzzy Level to \c level .
    void set_value(
            const T& new_value,
            FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

    //! Set Fuzzy Level to \c level .
    void set_level(
            FuzzyLevelType level = FuzzyLevelValues::fuzzy_level_set);

protected:

    /////////////////////////
    // VARIABLES
    /////////////////////////

    /**
     * @brief Fuzzy level of this object.
     *
     * This defines the certainty with which the internal \c value has been set.
     * By default is set to \c fuzzy_level_default if the internal value has not been set.
     */
    FuzzyLevelType fuzzy_level = FuzzyLevelValues::fuzzy_level_default;

    /**
     * @brief Internal value of type \c T .
     *
     * By default this value would be initialized with default constructor.
     */
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
