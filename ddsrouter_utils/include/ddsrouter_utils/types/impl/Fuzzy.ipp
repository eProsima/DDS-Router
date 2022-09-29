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
        FuzzyLevelType level /* = FuzzyLevelValues::set */)
    : value(other)
    , fuzzy_level(level)
{
}

template <typename T>
Fuzzy<T>::Fuzzy(
        T&& other,
        FuzzyLevelType level /* = FuzzyLevelValues::set */)
    : value(std::move(other))
    , fuzzy_level(level)
{
}

/////////////////////////
// OPERATORS
/////////////////////////

template <typename T>
Fuzzy<T>::operator T&() noexcept
{
    return value;
}

template <typename T>
Fuzzy<T>::operator T() const noexcept
{
    return value;
}

template <typename T>
bool Fuzzy<T>::operator ==(
        const Fuzzy<T>& other) const noexcept
{
    // If both unset, the object is the same
    if (!this->is_set() && !other.is_set())
    {
        return true;
    }
    else
    {
        return this->fuzzy_level == other.fuzzy_level && this->value == other.get_reference();
    }
}

template <typename T>
bool Fuzzy<T>::operator ==(
        const T& other) const noexcept
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
bool Fuzzy<T>::operator !=(
        const Fuzzy<T>& other) const noexcept
{
    return !(this->operator ==(other));
}

template <typename T>
bool Fuzzy<T>::operator !=(
        const T& other) const noexcept
{
    return !(this->operator ==(other));
}

/////////////////////////
// GET METHODS
/////////////////////////

template <typename T>
bool Fuzzy<T>::is_valid() const noexcept
{
    return fuzzy_level >= FuzzyLevelValues::fuzzy_level_default;
}

template <typename T>
bool Fuzzy<T>::is_set() const noexcept
{
    return fuzzy_level >= FuzzyLevelValues::fuzzy_level_fuzzy;
}

template <typename T>
T& Fuzzy<T>::get_reference() noexcept
{
    return value;
}

template <typename T>
const T& Fuzzy<T>::get_reference() const noexcept
{
    return value;
}

template <typename T>
T Fuzzy<T>::get_value() const noexcept
{
    return value;
}

template <typename T>
FuzzyLevelType Fuzzy<T>::get_level() const noexcept
{
    return fuzzy_level;
}

/////////////////////////
// SET METHODS
/////////////////////////

template <typename T>
void Fuzzy<T>::unset()
{
    // It is not needed to change value, as it would be used as it is not set
    fuzzy_level = FuzzyLevelValues::fuzzy_level_unset;
}

template <typename T>
void Fuzzy<T>::set_value(
        const T& new_value,
        FuzzyLevelType level /* = FuzzyLevelValues::SET */)
{
    value = new_value;
    fuzzy_level = level;
}

template <typename T>
void Fuzzy<T>::set_level(
        FuzzyLevelType level /* = FuzzyLevelValues::fuzzy_level_set */)
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
    os << "Fuzzy{Level(" << f.get_level() << ") " << f.get_reference() << "}";
    return os;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TYPES_FUZZY_IMPL_IPP_ */
