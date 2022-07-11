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
 * @file OwnerPtr.ipp
 */

#ifndef _DDSROUTERUTILS_IMPL_MEMORY_LESSEEPTR_IPP_
#define _DDSROUTERUTILS_IMPL_MEMORY_LESSEEPTR_IPP_

#include <ddsrouter_utils/exception/ValueAccessException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

////////////////////////////
// PROTECTED CONSTRUCTOR
////////////////////////////

template<typename T>
LesseePtr<T>::LesseePtr(
        std::shared_ptr<InternalPtrData<T>> data_reference)
    : data_reference_(data_reference)
{
}

///////////////////////
// CONSTRUCTORS
///////////////////////

template<typename T>
LesseePtr<T>::LesseePtr()
    : data_reference_()
{
}

template<typename T>
LesseePtr<T>::~LesseePtr()
{
    // Do nothing. Mutex is handled by GuardPtr and internal data is shared
}

template<typename T>
LesseePtr<T>::LesseePtr(
        const LesseePtr<T>& other)
{
    this->data_reference_ = other.data_reference_;
}

template<typename T>
LesseePtr<T>::LesseePtr(
        LesseePtr<T>&& other)
{
    this->data_reference_ = std::move(other.data_reference_);
    // Move a shared ptr reset the internal ptr
}

template<typename T>
LesseePtr<T>& LesseePtr<T>::operator =(
        const LesseePtr<T>& other)
{
    this->data_reference_ = other.data_reference_;
    this->shared_mutex_ = other.shared_mutex_;

    return *this;
}

template<typename T>
LesseePtr<T>& LesseePtr<T>::operator =(
        LesseePtr<T>&& other)
{
    this->data_reference_ = std::move(other.data_reference_);
    // Move a shared ptr reset the internal ptr

    return *this;
}

///////////////////////
// ACCESS DATA METHODS
///////////////////////

template<typename T>
T* LesseePtr<T>::operator ->()
{
    return this->lock_with_exception().operator ->();
}

template<typename T>
T& LesseePtr<T>::operator *()
{
    return this->lock_with_exception().operator *();
}

template<typename T>
GuardedPtr<T> LesseePtr<T>::lock() noexcept
{
    return lock_(false);
}

template<typename T>
GuardedPtr<T> LesseePtr<T>::lock_with_exception()
{
    return lock_(true);
}

template<typename T>
LesseePtr<T>::operator bool() const noexcept
{
    return data_reference_ && data_reference_->operator bool();
}

////////////////////////////
// AUXILIARY METHODS
////////////////////////////

template<typename T>
GuardedPtr<T> LesseePtr<T>::lock_(
        bool throw_exception)
{
    if (data_reference_)
    {
        // Lock mutex
        data_reference_->lock_shared();
        // From here, the validity cannot change until releasing lock

        // If data inside is not valid and exception must be thrown
        if (throw_exception && !data_reference_->operator bool())
        {
            // It will not be unlocked by the GuardedPtr, so unlock here
            data_reference_->unlock_shared();

            throw ValueAccessException(
                        "Trying to access a data not available anymore.");
        }
    }

    // Create a GuardedPtr that will unlock mutex in destruction
    // It could be that data is not valid, and this GuardedPtr will be invalid, but still exist
    // with a valid shared_ptr to an empty data.
    // If data_reference_ is nullptr, Guarded will not be valid and so it will not unlock mutex.
    return GuardedPtr<T> (
        data_reference_);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_MEMORY_LESSEEPTR_IPP_ */
