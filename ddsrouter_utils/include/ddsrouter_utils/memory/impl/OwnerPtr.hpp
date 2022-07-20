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
 * @file OwnerPtr.hpp
 *
 * This file contains class OwnerPtr definition.
 */

#ifndef _DDSROUTERUTILS_MEMORY_IMPL_OWNERPTR_HPP_
#define _DDSROUTERUTILS_MEMORY_IMPL_OWNERPTR_HPP_

#include <memory>
#include <mutex>
#include <shared_mutex>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/memory/impl/InternalPtrData.hpp>
#include <ddsrouter_utils/memory/impl/LesseePtr.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * Class that contains a reference (ptr) for an object whose ownership is \c this .
 * When \c this is destroyed, the object is destroyed too (using the deleter function given).
 * In order to use the object from another context, it must be by locking a \c LesseePtr ,
 * that is allowed to use it and assures the object is not destroyed while it is being used.
 * But LesseePtr does not own the object,
 * which could only be destroyed when no lessee is using it (and thus locking it).
 *
 * @tparam T Type of the data referenced by this ptr.
 */
template <class T>
class OwnerPtr final
{
public:

    ///////////////////////
    // CONSTRUCTORS
    ///////////////////////

    /**
     * @brief Construct a new empty Owner Ptr object
     *
     * An empty object does not contain any data.
     */
    OwnerPtr();

    /**
     * @brief Construct a new Owner Ptr object
     *
     * @param reference
     * @param deleter
     */
    OwnerPtr(
            T* reference,
            std::function<void(T*)> deleter = default_deleter());

    /**
     * @brief Destroy the Owner Ptr object
     *
     * It waits in case there are locks taken from the lessees.
     * Once every lessee has not the data locked, it destroys it and destroys itself.
     */
    ~OwnerPtr();

    //! This class is not copyable
    OwnerPtr(
            const OwnerPtr<T>& other) = delete;

    //! Movement constructor
    OwnerPtr(
            OwnerPtr<T>&& other);

    //! This class is not copyable
    OwnerPtr<T>& operator =(
            const OwnerPtr<T>& other) = delete;

    //! Movement operator
    OwnerPtr<T>& operator =(
            OwnerPtr<T>&& other);

    ///////////////////////
    // INTERACTION METHODS
    ///////////////////////

    /**
     * @brief Create a new Lessee object that references the data owned by this object.
     */
    LesseePtr<T> lease();

    /**
     * @brief Destroys the data owned by this object without destroying the object itself.
     *
     * This makes this object to have no data, and it releases the old data.
     * Thus, it must wait if the data is locked by lessees.
     *
     * Using this method, the lessees created so far will be detached (this is, the mutex is no longer shared).
     */
    void reset();

    /**
     * @brief Destroys the data owned by this object and uses the new reference as new data.
     *
     * This makes this object to destroy the old data.
     * Thus, it must wait if the data is locked by lessees.
     *
     * Using this method, the lessees created so far will be detached (this is, the mutex is no longer shared).
     */
    void reset(
            T* reference,
            std::function<void(T*)> deleter = default_deleter());

    ///////////////////////
    // ACCESS DATA METHODS
    ///////////////////////

    /**
     * @brief Access the data owned by this object with operator->
     *
     * @warning this method does not check if the data is valid.
     */
    T* operator ->();

    /**
     * @brief Access to a reference of the data owned by this object.
     *
     * @warning this method does not check if the data is valid.
     * @warning this method does not block this reference, that could be destroyed while used.
     */
    T& operator *();

    /**
     * @brief Access to raw ptr of the data
     *
     * @warning this method does not check if the data is valid.
     * @warning this method does not block this reference, that could be destroyed while used.
     *
     * @return T* raw ptr to data
     */
    T* get();

    /**
     * @brief Whether the data is not a nullptr.
     *
     * @warning this method does not know if internal data is accessible, only if it is not a nullptr.
     */
    operator bool() const noexcept;

    ////////////////////////////
    // STATIC AUXILIARY METHODS
    ////////////////////////////

    /**
     * @brief Default deleter lambda used if deleter is not given
     *
     * This deleter uses \c delete to destroy the internal data.
     * This is commonly used if the data is passed to this ptr by using \c new .
     */
    static std::function<void(T*)> default_deleter();

protected:

    ////////////////////////////
    // INTERNAL VARIABLES
    ////////////////////////////
    /**
     * @brief Shared internal data that will be shared between this owner and every lessee (and guard) using it.
     *
     * It could be nullptr in case the data is not valid (it has been reset or has been constructed without ptr).
     */
    std::shared_ptr<InternalPtrData<T>> data_reference_;

    //////////////////////////////
    // STATIC AUXILIARY VARIABLES
    //////////////////////////////

    //! Default deleter lambda
    static const std::function<void(T*)> DEFAULT_DELETER_;
};

////////////////////////////
// EXTERNAL OPERATORS
////////////////////////////

//! Allow to compare an OwnerPtr with nullptr
template<class T>
bool operator ==(
        const OwnerPtr<T>& lhs,
        std::nullptr_t) noexcept;

//! Allow to compare an OwnerPtr with nullptr in the other direction (from C++20 this is not needed)
template<class T>
bool operator ==(
        std::nullptr_t,
        const OwnerPtr<T>& lhs) noexcept;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/memory/impl/OwnerPtr.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_IMPL_OWNERPTR_HPP_ */
