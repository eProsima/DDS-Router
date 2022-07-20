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
 * @file InternalPtrData.hpp
 *
 * This file contains class InternalPtrData definition.
 */

#ifndef _DDSROUTERUTILS_MEMORY_IMPL_INTERNALPTRDATA_HPP_
#define _DDSROUTERUTILS_MEMORY_IMPL_INTERNALPTRDATA_HPP_

#include <memory>
#include <mutex>
#include <shared_mutex>

#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Forward declaration of OwnerPtr to use it as friendly class in LesseePtr
template <class T>
class OwnerPtr;

/**
 * @brief This is a data collection used to share information between the different pointers.
 *
 * This will be used in a shared pointer to share information between the pointers, and will never be nullptr.
 * Its validity is checked by checking the internal ptr is or is not nullptr.
 *
 * @note It could only be created with data and dereferenced from OwnerPtr.
 *
 * @tparam T Type of the internal data.
 */
template <class T>
class InternalPtrData
{
public:

    ///////////////////////
    // CONSTRUCTORS
    ///////////////////////

    //! Create an empty (non valid) data
    InternalPtrData();

    //! Move constructor
    InternalPtrData(InternalPtrData&& other);

    /**
     * @brief Destruct object
     *
     * It should not happen, but in case the ptr is still valid, it will be released.
     */
    ~InternalPtrData();

    ///////////////////////
    // INTERACTION METHODS
    ///////////////////////

    //! Lock this data in a shared way
    void lock_shared();

    //! Unlock this data in a shared way
    void unlock_shared();

    ///////////////////////
    // ACCESS DATA METHODS
    ///////////////////////

    //! Access to operator-> of the internal data (do not check if the internal data is valid)
    T* operator ->();

    //! Access to operator * of the internal data (do not check if the internal data is valid)
    T& operator *();

    //! Access to raw ptr of the data (do not check if the internal data is valid)
    T* get();

    //! Check if the internal data is valid (it is not nullptr)
    operator bool() const noexcept;

protected:

    //////////////////////////////////
    // PROTECTED METHODS FOR OWNERPTR
    //////////////////////////////////

    //! It requires friendship to use the constructor and \c dereference method.
    friend class OwnerPtr<T>;

    /**
     * @brief Construct a new Internal Ptr Data object with a ptr to the data and a specific deleter.
     *
     * This could only be constructed from OwnerPtr.
     *
     * @param reference Pointer to the data.
     * @param deleter Deleter to use when dereferencing the data.
     */
    InternalPtrData(
        T* reference,
        const std::function<void(T*)>& deleter);

    /**
     * @brief Delete the internal data
     *
     * It deletes the internal ptr using the specific deleter and locking the mutex (unique lock) so
     * it assures no other ptr is using the data at that time.
     */
    void release_reference_();

    ////////////////////////////
    // INTERNAL VARIABLES
    ////////////////////////////

    //! Pointer to the internal data
    T* reference_;

    //! Mutex to guard the data while using or destroying it
    std::shared_timed_mutex shared_mutex_;

    //! Deleter to use when dereferencing the data
    std::function<void(T*)> deleter_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/memory/impl/InternalPtrData.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_IMPL_INTERNALPTRDATA_HPP_ */
