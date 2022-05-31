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
 * This file contains class OwnerPtr and LesseePtr implementation.
 */

#ifndef _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_
#define _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_

#include <memory>
#include <mutex>
#include <shared_mutex>

#include <ddsrouter_utils/library/library_dll.h>


/**
 * @brief These classes implement a new smart pointer that allows to reference data with only one ownership
 * from different scopes and threads, in a way that access is secured and forces the data to exist while using it.
 *
 * INSTRUCTIONS
 * - OWNER PTR
 * The correct way of using it is having a \c OwnerPtr object referencing the data.
 * This is the only object that will be able to create and destruct such data (has ownership).
 * This object can destroy the data at any time (by deleting object or \c reset method).
 * This object will delete the internal data by a specific deleter given (if no deleter given, use \c delete )
 * If deleting data occurs while a sub object is USING (not handling) the data, this object will wait (in a mutex)
 * until it is safe to erase it.
 *
 * - LESSEE PTR
 * An \c OwnerPtr can lease its internal data to one or multiple \c LesseePtr objects.
 * This object can access the data by operators -> and * and will throw an exception in case the data is not
 * valid anymore (it has been deleted from the owner). While using the data, the owner will not be able to remove it.
 * This object can also "lock" the data, creating a \c GuardedPtr that will force the data to not be removed
 * while the object exist.
 *
 * - GUARDED PTR
 * This object force the internal data to exist while this object exists (locks de data destruction).
 * Thus, it is recommended to use this object as soon as possible and remove it afterwards, letting the ownership
 * again to the owner of the data.
 *
 * DATA PROTECTION
 * The data is protected by a shared mutex in a way that the owner can only remove it when no one is using it.
 * It is important to notice that this mutex does not protect access to data, it only protects it from destruction.
 * The shared mutex is used in shared mode from Lessee so that the data can be used by multiple threads.
 * The shared mutex is locked uniquely from the Owner so no Lessee can use it while destruction.
 *
 * USE CASE
 * Whenever an object must be created and deleted from a single point, but could be used from different
 * scopes and/or threads.
 *
 * WHY NEW CLASS
 * It is similar to a shared_ptr and weak_ptr, but this assures the object is only destroyed from its
 * owner, and only if no lessee is using it at the moment.
 *
 * IMPLEMENTATION
 * The way it works, is that every \c OwnerPtr shares a mutex with every \c LesseePtr , so while the
 * data is being used by a lessee, the owner cannot destroy it.
 * Whenever the object is used from a lessee, a new ptr is created and the mutex locked. When this ptr is deleted
 * the mutex is unlocked (by unique_ptr deleter).
 *
 * EXAMPLE OF USAGE
 *
 * - By Guarded Ptr (recommended)
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
 *
 * - By Lessee Ptr and exceptions
 * OwnerPtr<T> owner(new T());
 * LesseePtr<T> lessee = owner.lease();
 * {
 *   try
 *   {
 *     lessee->foo(); // Data wont be destroyed while foo is calling. It could be destroyed afterwards
 *   }
 *   catch (const eprosima::ddsrouter::utils::ValueAccessException& e)
 *   {
 *    // Ups, the data is not available anymore
 *   }
 * }
 */

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Forward declaration of OwnerPtr to use it as friendly class in LesseePtr
template <class T>
class OwnerPtr;

//! Forward declaration of LesseePtr to use it as friendly class in GuardedPtr
template <class T>
class LesseePtr;

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

    //! Create an empty (non valid) data
    InternalPtrData();

    //! Move constructor
    InternalPtrData(InternalPtrData&& other);

    //! Lock this data in a shared way
    void lock_shared();
    //! Unlock this data in a shared way
    void unlock_shared();

    //! Access to operator-> of the internal data (do not check if the internal data is valid)
    T* operator ->();
    //! Access to operator * of the internal data (do not check if the internal data is valid)
    T& operator *();
    //! Access to raw ptr of the data (do not check if the internal data is valid)
    T* get();

    //! Check if the internal data is valid (it is not nullptr)
    operator bool() const noexcept;

protected:

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
    void dereference();

    //! Pointer to the internal data
    T* reference_;

    //! Mutex to guard the data while using or destroying it
    std::shared_timed_mutex shared_mutex_;

    //! Deleter to use when dereferencing the data
    std::function<void(T*)> deleter_;
};

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

    //! Secure access to internal ptr as long as this object is valid
    T* operator ->();

    //! Secure access to internal ptr as long as this object is valid
    T& operator *();
    /**
     * @brief Access to raw ptr of the data (do not check if the internal data is valid)
     *
     * @warning This method is NOT RECOMMENDED as the idea of this object is to exist the minimum possible
     * and using the ptr outside this object could left the ptr unprotected.
     *
     * @return T* raw ptr to data
     */
    T* get();

    //! Check whether the internal ptr is valid
    operator bool() const noexcept;

protected:

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

    //! Internal shared data
    std::shared_ptr<InternalPtrData<T>> data_reference_;

};

//! Allow to compare an GuardedPtr with nullptr
template<class T>
bool operator ==(
        const GuardedPtr<T>& lhs,
        std::nullptr_t) noexcept;

//! Allow to compare an GuardedPtr with nullptr in the other direction (from C++20 this is not needed)
template<class T>
bool operator ==(
        std::nullptr_t,
        const GuardedPtr<T>& lhs) noexcept;

/**
 * This class contains a reference (ptr) to a value of type T but does not own it.
 * Not owning the object implies that, when being used, this reference could have been invalidated (destroyed).
 * In order to avoid that, method \c lock creates a smart pointer referencing that object, that will not be destroyed
 * while the pointer exists.
 *
 * @tparam T Type of the data referenced by this ptr.
 *
 * @note It could only be created from a \c OwnerPtr object, the owner of the data referenced.
 */
template <class T>
class LesseePtr final
{
public:

    /**
     * @brief Construct a new empty Lessee Ptr object
     *
     * This constructor is only used to create an empty object.
     * It could not contain any data.
     */
    LesseePtr();

    /**
     * @brief Destroy the Lessee Ptr object
     *
     * It does not destroy the data (it has not the ownership).
     */
    ~LesseePtr();

    //! Copy constructor from \c OwnerPtr
    LesseePtr(
            const OwnerPtr<T>& owner);

    //! Copy constructor
    LesseePtr(
            const LesseePtr<T>& other);

    // Move constructor
    LesseePtr(
            LesseePtr<T>&& other);

    T* operator ->();

    T& operator *();

    /**
     * @brief copy assigment
     *
     * It copies the reference from the other object.
     *
     * @param other object to copy
     * @return this object
     */
    LesseePtr<T>& operator =(
            const LesseePtr<T>& other);

    /**
     * @brief move assigment
     *
     * It gets the reference from the other object.
     * \c other object loses its internal data and gets invalidated.
     *
     * @param other object to copy
     * @return this object
     */
    LesseePtr<T>& operator =(
            LesseePtr<T>&& other);

    /**
     * @brief Create a smart reference to the data.
     *
     * While the smart reference (return of this method) exists, the data cannot be destroyed.
     * This smart reference must be destroyed as soon as possible, because it locks the real owner of the data.
     *
     * @note This method is similar to \c std::weak_ptr::lock() .
     *
     * @warning return of this method must be checked before used, the data returned could not be valid anymore.
     * @warning this method does not protect the access to the internal data. It only avoids its destruction.
     *
     * @return std::shared_ptr<T> to the data. nullptr if the reference is not valid anymore.
     */
    GuardedPtr<T> lock() noexcept;

    /**
     * @brief Create a smart reference to the data or throw an exception if data is not available
     *
     * While the smart reference (return of this method) exists, the data could not be destroyed.
     * This smart reference must be destroyed as soon as possible, because it locks the real owner of the data.
     *
     * Use this method instead of \c lock if the non existence of the data is treated as error.
     *
     * @warning this method does not protect the access to the internal data. It only avoids its destruction.
     *
     * @return std::shared_ptr<T> to the data
     * @throw \c InitializationException if the data is not valid anymore.
     */
    GuardedPtr<T> lock_with_exception();

    operator bool() const noexcept;

protected:

    // It requires friendship to use the constructor
    friend class OwnerPtr<T>;

    /**
     * @brief Construct a new Lessee Ptr object
     *
     * Protected constructor that must be called from \c OwnerPtr .
     *
     * @param data weak reference to the data.
     * @param shared_mutex shared mutex between owner and this.
     */
    LesseePtr(
            std::shared_ptr<InternalPtrData<T>> data_reference);

    /**
     * @brief Generic lock method that throws an exception or return nullptr depending on the argument.
     *
     * @param throw_exception whether the method must throw an exception in case of error
     * @return std::shared_ptr<T> to the data. nullptr if the reference is not valid anymore if not \c throw_exception .
     * @throw \c InitializationException if the data is not valid anymore if \c throw_exception is \c true .
     */
    GuardedPtr<T> lock_(
            bool throw_exception);

    std::shared_ptr<InternalPtrData<T>> data_reference_;

};

/**
 * Class that contains a reference (ptr) for an object whose ownership is \c this .
 * When \c this is destroyed, the object is destroyed too (using the deleter function given).
 * In order to use the object from another context, it must be by using a \c LesseePtr ,
 * that is allowed to use it and assures the object is not destroyed while it is being used.
 * But LesseePtr does not own the object, which could only be destroyed when no lessee is using it.
 *
 * @tparam T Type of the data referenced by this ptr.
 */
template <class T>
class OwnerPtr final
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
    T* operator ->();

    /**
     * @brief Access to a reference of the data owned by this object.
     *
     * @warning this method does not check if the data is valid.
     * @warning this method does not block this reference, that could be destroyed while used.
     */
    T& operator *();

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

    /**
     * @brief Shared internal data that will be shared between this owner and every lessee (and guard) using it.
     *
     * It could be nullptr in case the data is not valid (it has been reset or has been constructed without ptr).
     */
    std::shared_ptr<InternalPtrData<T>> data_reference_;

    //! Default deleter lambda
    static const std::function<void(T*)> DEFAULT_DELETER_;
};

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
#include <ddsrouter_utils/memory/impl/InternalPtrData.ipp>
#include <ddsrouter_utils/memory/impl/GuardedPtr.ipp>
#include <ddsrouter_utils/memory/impl/LesseePtr.ipp>
#include <ddsrouter_utils/memory/impl/OwnerPtr.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_ */
