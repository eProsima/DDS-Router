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

#pragma once

#ifndef DDSROUTERUTILS_TYPES_FUZZY_HPP_
#define DDSROUTERUTILS_TYPES_FUZZY_HPP_

#include <ostream>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/macros/custom_enumeration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

ENUMERATION_BUILDER(
    FuzzyLevel,
    unset,
    default_set,
    fuzzy_set,
    set
);

/**
 * TODO
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
        FuzzyLevel level = FuzzyLevel::set);

    Fuzzy(
        T&& other,
        FuzzyLevel level = FuzzyLevel::set);

    /////////////////////////
    // OPERATORS
    /////////////////////////

    operator T() const noexcept;

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

    void set_value(const T& new_value, FuzzyLevel level = FuzzyLevel::set);

    /////////////////////////
    // VARIABLES
    /////////////////////////

    FuzzyLevel fuzzy_level = FuzzyLevel::default_set;

    T value;
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

#endif /* DDSROUTERUTILS_TYPES_FUZZY_HPP_ */
