// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License")
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
 * @file Fuzzy.ipp
 *
 */

#ifndef _DDSROUTERUTILS_TYPES_FUZZY_IMPL_IPP_
#define _DDSROUTERUTILS_TYPES_FUZZY_IMPL_IPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
Fuzzy<T>::Fuzzy(
        const T& other,
        FuzzyLevel level /* = FuzzyLevel::SET */)
    : value(other)
    , fuzzy_level(level)
{
}

template <typename T>
Fuzzy<T>::Fuzzy(
        T&& other,
        FuzzyLevel level /* = FuzzyLevel::SET */)
    : value(std::move(other))
    , fuzzy_level(level)
{
}

// template <typename T>
// Fuzzy<T>::Fuzzy(
//         const Fuzzy<T>& other)
//     : value(other.value)
//     , fuzzy_level(other.fuzzy_level)
// {
// }

// template <typename T>
// Fuzzy<T>::Fuzzy(
//         Fuzzy<T>&& other)
//     : value(std::move(other.value))
//     , fuzzy_level(other.fuzzy_level)
// {
// }

// template <typename T>
// Fuzzy<T>& Fuzzy<T>::operator=(const Fuzzy<T>& other) noexcept
// {
//     this->value = other.value;
//     this->fuzzy_level = other.fuzzy_level;
//     return *this;
// }

// template <typename T>
// Fuzzy<T>& Fuzzy<T>::operator=(const T& other) noexcept
// {
//     this->value = other;
//     this->fuzzy_level = FuzzyLevel::set;
//     return *this;
// }

// template <typename T>
// Fuzzy<T>& Fuzzy<T>::operator=(Fuzzy<T>&& other) noexcept
// {
//     this->value = std::move(other.value);
//     this->fuzzy_level = other.fuzzy_level;
//     return *this;
// }

// template <typename T>
// Fuzzy<T>& Fuzzy<T>::operator=(T&& other) noexcept
// {
//     this->value = std::move(other);
//     this->fuzzy_level = FuzzyLevel::set;
//     return *this;
// }

/////////////////////////
// OPERATORS
/////////////////////////

// template <typename T>
// Fuzzy<T>::operator T() const noexcept
// {
//     return value;
// }

template <typename T>
Fuzzy<T>::operator const T&() const noexcept
{
    return value;
}

template <typename T>
bool Fuzzy<T>::operator==(const Fuzzy<T>& other) const noexcept
{
    // If both unset, the object is the same
    if (this->fuzzy_level == FuzzyLevel::unset && this->fuzzy_level == other.fuzzy_level)
    {
        return true;
    }
    else
    {
        return this->fuzzy_level == other.fuzzy_level || this->value == other.value;
    }
}

template <typename T>
bool Fuzzy<T>::operator==(const T& other) const noexcept
{
    if (!this->is_valid())
    {
        return false;
    }
    else
    {
        return this->value == other;
    }
}

template <typename T>
bool Fuzzy<T>::operator!=(const Fuzzy<T>& other) const noexcept
{
    return !(this->operator==(other));
}

template <typename T>
bool Fuzzy<T>::operator!=(const T& other) const noexcept
{
    return !(this->operator==(other));
}

/////////////////////////
// GET METHODS
/////////////////////////

template <typename T>
bool Fuzzy<T>::is_valid() const noexcept
{
    return fuzzy_level != FuzzyLevel::unset;
}

template <typename T>
bool Fuzzy<T>::is_set() const noexcept
{
    return fuzzy_level == FuzzyLevel::set || fuzzy_level == FuzzyLevel::fuzzy_set;
}

template <typename T>
const T& Fuzzy<T>::get_reference() const noexcept
{
    return value;
}

/////////////////////////
// SET METHODS
/////////////////////////

template <typename T>
void Fuzzy<T>::set_value(const T& new_value, FuzzyLevel level /* = FuzzyLevel::SET */)
{
    value = new_value;
    fuzzy_level = level;
}

template <typename T>
void Fuzzy<T>::check_as_set(FuzzyLevel level /* = FuzzyLevel::set */)
{
    fuzzy_level = level;
}


/////////////////////////
// SERIALIZATION
/////////////////////////

template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const Fuzzy<T>& f)
{
    if (f.is_set())
    {
        os << f.value;
    }
    else
    {
        os << "{" << f.fuzzy_level << ";" << f.value << "}";
    }
    return os;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TYPES_FUZZY_IMPL_IPP_ */
