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
 * @file Formatter.ipp
 */

#ifndef _DDSROUTERUTILS_IMPL_MEMORY_OWNERPTR_IPP_
#define _DDSROUTERUTILS_IMPL_MEMORY_OWNERPTR_IPP_

#include <ddsrouter_utils/exception/InitializationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template<typename T>
LesseePtr<T>::LesseePtr(
        std::weak_ptr<T> data,
        std::shared_ptr<std::mutex> shared_mutex)
    : data_reference_(data)
    , shared_mutex_(shared_mutex)
{
}

template<typename T>
LesseePtr<T>::~LesseePtr()
{
}

template<typename T>
std::shared_ptr<T> LesseePtr<T>::lock()
{
    shared_mutex_->lock();

    std::shared_ptr<T> locked_ptr = data_reference_.lock();

    if (!locked_ptr)
    {
        this->shared_mutex_->unlock();
        return nullptr;
    }

    // Create a different shared_ptr that points to the same element
    return std::shared_ptr<T> (
        locked_ptr.get(),
        [this](T* ptr)
        {
            this->shared_mutex_->unlock();
        });
}

template<typename T>
const std::function<void(T*)> OwnerPtr<T>::DEFAULT_DELETER_ = [](T* value){ delete value; };

template<typename T>
OwnerPtr<T>::OwnerPtr()
{
}

template<typename T>
OwnerPtr<T>::OwnerPtr(
        T* reference,
        std::function<void(T*)> deleter /* = default_deleter() */)
{
    if (nullptr == reference)
    {
        throw InitializationException(
            "Trying to create an OwnerPtr without a nullptr.");
    }
    else
    {
        data_reference_ = std::shared_ptr<T>(reference, deleter);
    }
}

template<typename T>
OwnerPtr<T>::~OwnerPtr()
{
    reset();
}

template<typename T>
LesseePtr<T> OwnerPtr<T>::lease()
{
    std::shared_ptr<std::mutex> new_mutex = std::make_shared<std::mutex>();

    leases_mutexes_.push_back(new_mutex);

    return LesseePtr<T>(
        data_reference_,
        new_mutex);
}

template<typename T>
void OwnerPtr<T>::reset()
{
    for (std::shared_ptr<std::mutex> mutex : leases_mutexes_)
    {
        mutex->lock();
    }

    data_reference_.reset();

    for (std::shared_ptr<std::mutex> mutex : leases_mutexes_)
    {
        mutex->unlock();
    }

    leases_mutexes_.clear();
}

template<typename T>
void OwnerPtr<T>::reset(
        T* reference,
        std::function<void(T*)> deleter /* = default_deleter() */)
{
    reset();

    if (nullptr == reference)
    {
        throw InitializationException(
            "Trying to reset an OwnerPtr with a nullptr.");
    }
    else
    {
        data_reference_ = std::shared_ptr<T>(reference, deleter);
    }
}

template<typename T>
T* OwnerPtr<T>::operator->()
{
    return data_reference_.operator->();
}

template<typename T>
T& OwnerPtr<T>::operator*()
{
    return data_reference_.operator*();
}

template<typename T>
OwnerPtr<T>::operator bool() const noexcept
{
    return data_reference_.operator bool();
}

template<typename T>
std::function<void(T*)> OwnerPtr<T>::default_deleter()
{
    return OwnerPtr<T>::DEFAULT_DELETER_;
}

template<class T>
bool operator==(const OwnerPtr<T>& lhs, std::nullptr_t) noexcept
{
    return !lhs;
}

template<class T>
bool operator==(std::nullptr_t, const OwnerPtr<T>& lhs) noexcept
{
    return !lhs;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_MEMORY_OWNERPTR_IPP_ */
