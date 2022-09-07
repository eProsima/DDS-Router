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
 * @file AtomicableValue.ipp
 *
 * This file contains class Atomicable implementation.
 */

#ifndef __DDSROUTERUTILS_TYPES_IMPL_ATOMICABLEVALUE_IPP_
#define __DDSROUTERUTILS_TYPES_IMPL_ATOMICABLEVALUE_IPP_

#include <ddsrouter_utils/types/Atomicable.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
AtomicableValue<T>::AtomicableValue(const T& value)
    : value_(value)
{
}

template <typename T>
AtomicableValue<T>::AtomicableValue(T&& value)
    : value_(std::move(value))
{
}

template <typename T>
void AtomicableValue<T>::get(T& value_to_set) const noexcept
{
    std::shared_lock<SharedAtomicable<T>> lock(value_);
    value_to_set = value_;
}

template <typename T>
void AtomicableValue<T>::set(const T& new_value) const
{
    std::unique_lock<SharedAtomicable<T>> lock(value_);
    value_ = new_value;
}

template <typename T>
void AtomicableValue<T>::set(T&& new_value) const
{
    std::unique_lock<SharedAtomicable<T>> lock(value_);
    value = std::move(new_value);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __DDSROUTERUTILS_TYPES_IMPL_ATOMICABLEVALUE_IPP_ */
