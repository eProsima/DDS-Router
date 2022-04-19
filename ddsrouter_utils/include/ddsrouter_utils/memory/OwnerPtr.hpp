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
 * This file contains class OwnerPtr implementation.
 */

#ifndef _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_
#define _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_

#include <memory>
#include <mutex>

#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <class T>
class OwnerPtr;

/**
 * @brief This classes implement a new smart pointer that allows to reference data with only one ownership
 * from different scopes and threads.
 *
 * INSTRUCTIONS
 * The correct way of using it is having a \c OwnerPtr object referencing the data, and getting
 * a lessee from it.
 * Before using the data in the ptr, use \c lock and check the result is not null.
 * Destroy the ptr after using it. It is advisable to destroy it soon, as the owner could want to erase it.
 *
 * USE CASE
 * Whenever an object must be created and deleted from a single point, but could be used from different
 * scopes and/or threads.
 *
 * WHY NEW CLASS
 * It is similar to a shared_ptr and weak_ptr, but this assures the object is only destroyed from its
 * owner, and only if any lessee is not using it at the moment.
 *
 * IMPLEMENTATION
 * The way it works, is that every \c OwnerPtr shares a mutex with every \c LesseePtr , so while the
 * data is being used by a lessee, the owner could not destroy it.
 * Whenever the object is used from a lessee, a new ptr is created and mutex lock. When this ptr is deleted
 * the mutex is unlocked (by unique_ptr deleter).
 *
 *
 * EXAMPLE OF USAGE
 * OwnerPtr<T> owner(new T());
 * LesseePtr<T> lessee = owner.lease();
 * {
 *   auto locked_ptr_ = lessee.lock(); // From here till locked_ptr_ destruction, data could not be destroyed.
 *   if (locked_ptr_) {
 *     locked_ptr_->foo(); // Use data without worrying it could be destroyed
 *   } else {
 *     // Ups, the data is not available anymore
 *   }
 * }
 */

/**
 * This class contains a reference (ptr) to a value of type T but do not own it.
 * Not owning the object implies that, when using, this reference could have been invalidated.
 * For that, method \c lock create a smart pointer referencing that object, that will not be destroyed
 * while the smart pointer exists.
 *
 * @tparam T Type of the data referenced by this ptr.
 *
 * @note It could only be created from a \c OwnerPtr object, the owner of the data referenced.
 */
template <class T>
class LesseePtr
{
public:

    /**
     * @brief Construct a new empty Lessee Ptr object
     *
     * This constructor is only used to create an empty object.
     * It could not contain any data.
     * To use an empty ptr, use operator= .
     */
    LesseePtr();

    /**
     * @brief Destroy the Lessee Ptr object
     *
     * It waits in case there are locks taken.
     * It does not destroy the data (it has not the ownership).
     */
    ~LesseePtr();

    /**
     * @brief copy assigment
     *
     * It gets the reference from the other object.
     * It looses an old reference and the mutex in case it had it.
     *
     * @param other object to copy
     * @return LesseePtr<T>& this object
     */
    LesseePtr<T>& operator =(
            const LesseePtr<T>& other);

    /**
     * @brief Create an smart reference to the data.
     *
     * While the smart reference (return of this method) exists, the data could not be destroyed.
     * This smart reference must be destroyed as soon as possible, because it locks the real owner of the data.
     *
     * This method is similar to \c std::weak_ptr::lock() .
     *
     * @warning return of this method must be checked before used, the data returned could not be valid anymore.
     * @warning this method does not protect the access to the internal data. It only avoid its destruction.
     *
     * @return std::shared_ptr<T> to the data. nullptr if the reference is not valid anymore.
     */
    std::shared_ptr<T> lock() noexcept;

    /**
     * @brief Create an smart reference to the data or throw an exception if data is not available
     *
     * While the smart reference (return of this method) exists, the data could not be destroyed.
     * This smart reference must be destroyed as soon as possible, because it locks the real owner of the data.
     *
     * Use this method instead of \c lock if the non existence of the data is treated as error.
     *
     * @warning this method does not protect the access to the internal data. It only avoid its destruction.
     *
     * @return std::shared_ptr<T> to the data
     * @throw \c InitializationException if the data is not valid anymore.
     */
    std::shared_ptr<T> lock_with_exception();

protected:

    LesseePtr(
        std::weak_ptr<T> data,
        std::shared_ptr<std::mutex> shared_mutex);

    /**
     * @brief Generic lock method that throws an exception or return nullptr depending on the argument.
     *
     * @param throw_exception whether the method must throw an exception in case of error
     * @return std::shared_ptr<T> to the data. nullptr if the reference is not valid anymore if not \c throw_exception .
     * @throw \c InitializationException if the data is not valid anymore if \c throw_exception is \c true .
     */
    std::shared_ptr<T> lock_(bool throw_exception);

    // It requires friendship to use the constructor
    friend class OwnerPtr<T>;

    /**
     * @brief Reference to the data. The shared ptr that owns the data is in OwnerPtr
     *
     * This data must come from the \c OwnerPtr that has created this object.
     */
    std::weak_ptr<T> data_reference_;

    /**
     * @brief Reference to the mutex that must be locked to use the data.
     *
     * This mutex must come from the \c OwnerPtr that has created this object.
     * It is used to lock the data when it is being used.
     *
     * @note It is a shared_ptr because it is shared between the \c OwnerPtr and the \c LesseePtr.
     */
    std::shared_ptr<std::mutex> shared_mutex_;
};

/**
 * Class that contains a reference (ptr) for an object which ownership is \c this .
 * When \c this is destroyed, the object is destroyed too (using the deleter function given).
 * In order to use the object from another context, it must be by using a \c LesseePtr ,
 * that is allowed to use it and assures the object is not destroyed while it is being used, but do not
 * own the object, that could be destroyed when no lessee is using it.
 *
 * @tparam T Type of the data referenced by this ptr.
 */
template <class T>
class OwnerPtr
{
public:

    /**
     * @brief Construct a new empty Owner Ptr object
     *
     * An empty object does not contain any data.
     *
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
     * Once every lessee has not the data locked, it destroys it and destroy itself.
     */
    ~OwnerPtr();

    //! This class is not copyable
    OwnerPtr(const OwnerPtr<T>& other) = delete;

    //! This class is not copyable
    OwnerPtr<T>& operator =(
            const OwnerPtr<T>& other) = delete;

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
     * Using this method, the lessees created so far will be detached (this is, the mutex is not longer shared).
     */
    void reset();

    /**
     * @brief Destroys the data owned by this object and uses the new reference as new data.
     *
     * This makes this object to destroy the old data.
     * Thus, it must wait if the data is locked by lessees.
     *
     * Using this method, the lessees created so far will be detached (this is, the mutex is not longer shared).
     */
    void reset(
        T* reference,
        std::function<void(T*)> deleter = default_deleter());

    /**
     * @brief Access the data owned by this object with operator->
     *
     * @warning this method does not check if the data is valid.
     */
    T* operator->();

    /**
     * @brief Access to a reference of the data owned by this object.
     *
     * @warning this method does not check if the data is valid.
     * @warning this method does not block this reference, that could be destroyed while used.
     */
    T& operator*();

    /**
     * @brief Whether the data is not a nullptr.
     *
     * @warning this method does not know if internal data is accessible, only if it is not a nullptr.
     */
    operator bool() const noexcept;

    /**
     * @brief Default deleter lambda used if deleter is not given
     *
     * This deleter use \c delete to destroy the internal data.
     * This is commonly used if the data is passed to this ptr by using \c new .
     */
    static std::function<void(T*)> default_deleter();

protected:

    //! Reference to the data owned by this object (could be nullptr).
    std::shared_ptr<T> data_reference_;

    /**
     * @brief List of mutexes shared between this object and the lessees it has created.
     *
     * These mutexes will only be locked when the data is being used by a lessee, and only until the locked
     * result is alive.
     */
    std::vector<std::shared_ptr<std::mutex>> leases_mutexes_;

    //! Default deleter lambda
    static const std::function<void(T*)> DEFAULT_DELETER_;
};

//! Allow to compare an OwnerPtr with nullptr
template<class T>
bool operator==(const OwnerPtr<T>& lhs, std::nullptr_t) noexcept;

//! Allow to compare an OwnerPtr with nullptr in the other direction (from C++20 this is not needed)
template<class T>
bool operator==(std::nullptr_t, const OwnerPtr<T>& lhs) noexcept;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/memory/impl/OwnerPtr.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_ */
