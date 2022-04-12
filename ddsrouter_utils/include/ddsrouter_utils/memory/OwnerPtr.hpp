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
template <typename T>
class LesseePtr
{
public:

    ~LesseePtr();

    std::unique_ptr<T> lock();

protected:

    LesseePtr(
        std::shared_ptr<T> data,
        std::shared_ptr<std::mutex> shared_mutex);
    friend class OwnerPtr;

    //! Concatenated stream where the streams are added at the end
    const std::shared_ptr<T> data_reference_;
    const std::shared_ptr<std::mutex> shared_mutex_;
};

/**
 * Class that contains a reference (ptr) for an object hosted in \c this .
 * When \c this is destroyed, the object is destroyed too.
 * In order to use the object from another context, it must be by using a \c LesseePtr ,
 * that is allowed to use it and assures the object is not destroyed while it is being used, but do not
 * own the object, that could be destroyed when no lessee is using it.
 *
 * @tparam T Type of the data referenced by this ptr.
 */
template <typename T>
class OwnerPtr
{
public:

    OwnerPtr(T&& reference);

    ~OwnerPtr();

    LesseePtr<T> lease() const;

    T* operator->();

protected:

    const std::shared_ptr<T> data_reference_;
    std::vector<std::shared_ptr<std::mutex>> leases_mutexes_;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/memory/impl/OwnerPtr.ipp>

#endif /* _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_ */
