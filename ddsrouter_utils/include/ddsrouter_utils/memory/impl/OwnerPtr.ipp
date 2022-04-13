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

namespace eprosima {
namespace ddsrouter {
namespace utils {

template<typename T>
LesseePtr<T>::LesseePtr(
        std::shared_ptr<T> data,
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
std::unique_ptr<T> LesseePtr<T>::lock()
{
    shared_mutex_->lock();

    if (!data_reference_)
    {
        this->shared_mutex_->unlock();
        return nullptr;
    }

    return std::unique_ptr<T> (
        data_reference_.get(),
        [this](T* ptr)
        {
            this->shared_mutex_->unlock();
        });
}

template<typename T>
OwnerPtr<T>::OwnerPtr(
        T&& reference,
        std::function<void(T*)> deleter /* = [](T* value){ delete value; } */)
    : data_reference_(new T(reference), deleter)
{
}

template<typename T>
OwnerPtr<T>::~OwnerPtr()
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
T* OwnerPtr<T>::operator->()
{
    return data_reference_.operator->();
}

template<typename T>
T& OwnerPtr<T>::operator*()
{
    return data_reference_.operator*();
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_MEMORY_OWNERPTR_IPP_ */
