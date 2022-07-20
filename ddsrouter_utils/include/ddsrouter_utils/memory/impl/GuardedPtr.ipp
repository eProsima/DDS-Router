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
 * @file GuardedPtr.ipp
 */

#ifndef _DDSROUTERUTILS_IMPL_MEMORY_GUARDEDPTR_IPP_
#define _DDSROUTERUTILS_IMPL_MEMORY_GUARDEDPTR_IPP_

#include <ddsrouter_utils/exception/ValueAccessException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template<typename T>
GuardedPtr<T>::GuardedPtr(
        std::shared_ptr<InternalPtrData<T>> data_reference)
    : data_reference_(data_reference)
{
}

template<typename T>
GuardedPtr<T>::~GuardedPtr()
{
    // Avoid unlocking when no data is stored (only when created from a null)
    if (data_reference_)
    {
        data_reference_->unlock_shared();
    }
}

template<typename T>
GuardedPtr<T>::GuardedPtr(
        GuardedPtr&& other)
    : data_reference_(std::move(other.data_reference_))
{
}

template<typename T>
T* GuardedPtr<T>::operator ->()
{
    return data_reference_->operator ->();
}

template<typename T>
T& GuardedPtr<T>::operator *()
{
    return data_reference_->operator *();
}

template<typename T>
T* GuardedPtr<T>::get()
{
    return data_reference_->get();
}

template<typename T>
GuardedPtr<T>::operator bool() const noexcept
{
    return data_reference_ && data_reference_->operator bool();
}

template<class T>
bool operator ==(
        const GuardedPtr<T>& lhs,
        std::nullptr_t) noexcept
{
    return !lhs;
}

template<class T>
bool operator ==(
        std::nullptr_t,
        const GuardedPtr<T>& lhs) noexcept
{
    return !lhs;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_MEMORY_GUARDEDPTR_IPP_ */
