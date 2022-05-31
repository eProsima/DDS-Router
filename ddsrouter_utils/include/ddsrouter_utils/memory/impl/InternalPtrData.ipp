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
 * @file InternalPtrData.ipp
 */

#ifndef _DDSROUTERUTILS_IMPL_MEMORY_INTERNALPTRDATA_IPP_
#define _DDSROUTERUTILS_IMPL_MEMORY_INTERNALPTRDATA_IPP_

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/exception/ValueAccessException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template<typename T>
InternalPtrData<T>::InternalPtrData()
    : reference_(nullptr)
{
}

template<typename T>
InternalPtrData<T>::InternalPtrData(InternalPtrData&& other)
    : reference_(std::move(other.reference_))
    , shared_mutex_(std::move(other.shared_mutex_))
    , deleter_(std::move(other.deleter_))
{
}

template<typename T>
InternalPtrData<T>::InternalPtrData(T* reference, const std::function<void(T*)>& deleter)
    : reference_(reference)
    , deleter_(deleter)
{
}

template<typename T>
void InternalPtrData<T>::lock_shared()
{
    shared_mutex_.lock_shared();
}

template<typename T>
void InternalPtrData<T>::unlock_shared()
{
    shared_mutex_.unlock_shared();
}

template<typename T>
void InternalPtrData<T>::dereference()
{
    shared_mutex_.lock();
    if (reference_ != nullptr)
    {
        deleter_(reference_);
        reference_ = nullptr;
    }
    shared_mutex_.unlock();
}

template<typename T>
T* InternalPtrData<T>::operator ->()
{
    return reference_;
}

template<typename T>
T& InternalPtrData<T>::operator *()
{
    return *reference_;
}

template<typename T>
T* InternalPtrData<T>::get()
{
    return reference_;
}

template<typename T>
InternalPtrData<T>::operator bool() const noexcept
{
    return reference_ != nullptr;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_MEMORY_INTERNALPTRDATA_IPP_ */
