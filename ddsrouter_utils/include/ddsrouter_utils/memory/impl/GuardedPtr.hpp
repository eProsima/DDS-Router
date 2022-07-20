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
 * @file GuardedPtr.hpp
 *
 * This file contains class GuardedPtr definition.
 */

#ifndef _DDSROUTERUTILS_MEMORY_IMPL_GUARDEDPTR_HPP_
#define _DDSROUTERUTILS_MEMORY_IMPL_GUARDEDPTR_HPP_

#include <memory>
#include <mutex>
#include <shared_mutex>

#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Forward declaration of LesseePtr to use it as friendly class in GuardedPtr
template <class T>
class LesseePtr;

/**
 * @brief Simple Ptr object that only holds a pointer and a shared locked mutex.
 *
 * This is created from a \c LesserPtr and will release the mutex (once) when destroyed.
 *
 * @note It should always be created with a shared blocked mutex, as it will unlock mutex in destruction.
 *
 * @tparam T Type of the internal data.
 */
template <class T>
class GuardedPtr final
{
public:

    ///////////////////////
    // CONSTRUCTORS
    ///////////////////////

    //! Move constructor
    GuardedPtr(GuardedPtr&&);

    //! Copy constructor not allowed
    GuardedPtr(const GuardedPtr&) = delete;

    //! Copy operator not allowed
    GuardedPtr<T>& operator =(
            const GuardedPtr<T>& other) = delete;

    //! Move operator not allowed
    GuardedPtr<T>& operator =(
            GuardedPtr<T>&& other) = delete;

    //! Unlock the shared mutex
    ~GuardedPtr();

    ///////////////////////
    // ACCESS DATA METHODS
    ///////////////////////

    //! Secure access to internal ptr as long as this object is valid
    T* operator ->();

    //! Secure access to internal ptr as long as this object is valid
    T& operator *();

    /**
     * @brief Access to raw ptr of the data (do not check if the internal data is valid)
     *
     * @warning This method is NOT RECOMMENDED as the idea of this object is to exist the minimum possible
     * and using the ptr outside this object could leave the ptr unprotected.
     *
     * @return T* raw ptr to data
     */
    T* get();

    //! Check whether the internal ptr is valid
    operator bool() const noexcept;

protected:

    ///////////////////////////////////////
    // PROTECTED CONSTRUCTOR FOR LESSEEPTR
    ///////////////////////////////////////

    //! It requires friendship to use the constructor
    friend class LesseePtr<T>;

    /**
     * @brief Construct a new Guarded Ptr from a \c LesseePtr
     *
     * @warning \c data_reference must be shared locked before this creation
     *
     * @param data_reference
     */
    GuardedPtr(
            std::shared_ptr<InternalPtrData<T>> data_reference);

    ////////////////////////////
    // INTERNAL VARIABLES
    ////////////////////////////

    //! Internal shared data protected while this object exists
    std::shared_ptr<InternalPtrData<T>> data_reference_;

};

////////////////////////////
// EXTERNAL OPERATORS
////////////////////////////

//! Allow to compare a GuardedPtr with nullptr
template<class T>
bool operator ==(
        const GuardedPtr<T>& lhs,
        std::nullptr_t) noexcept;

//! Allow to compare a GuardedPtr with nullptr in the other direction (from C++20 this is not needed)
template<class T>
bool operator ==(
        std::nullptr_t,
        const GuardedPtr<T>& lhs) noexcept;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/memory/impl/GuardedPtr.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_IMPL_GUARDEDPTR_HPP_ */
