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
 * @file owner_ptr.hpp
 *
 * This file contains class OwnerPtr and LesseePtr implementation.
 */

#ifndef _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_
#define _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_

/**
 * @brief These classes implement a new smart pointer that allows to reference data with only one ownership
 * from different scopes and threads, in a way that access is secured and forces the data to exist while using it.
 *
 * INSTRUCTIONS
 * - OWNER PTR
 * The correct way of using is having a \c OwnerPtr object referencing the data.
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
 * while the object exists.
 *
 * - GUARDED PTR
 * This object forces the internal data to exist while this object exists (locks data destruction).
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
 * data is being blocked by a lessee, the owner cannot destroy it.
 * Whenever the object is used from a lessee, a new ptr is created and the mutex locked. When this ptr is deleted
 * the mutex is unlocked, and then the data could be destroyed.
 *
 * PROBLEMS
 * The main problem of this kind of object is that it could be blocked in destruction.
 *
 * EXAMPLE OF USAGE
 *
 * - By Guarded Ptr (recommended)
 *
 *   OwnerPtr<T> owner(new T());
 *   LesseePtr<T> lessee = owner.lease();
 *   {
 *     auto locked_ptr_ = lessee.lock(); // From here till locked_ptr_ destruction, data could not be destroyed.
 *     if (locked_ptr_) {
 *       locked_ptr_->foo(); // Use data without worrying, it could not be destroyed while locked_ptr_ exists.
 *     } else {
 *       // Ups, the data is not available anymore
 *     }
 *   }
 *
 *
 * - By Lessee Ptr and exceptions
 *
 *   OwnerPtr<T> owner(new T());
 *   LesseePtr<T> lessee = owner.lease();
 *   {
 *     try
 *     {
 *       lessee->foo(); // Data wont be destroyed while foo is calling. Note it could be destroyed just afterwards.
 *     }
 *     catch (const eprosima::ddsrouter::utils::ValueAccessException& e)
 *     {
 *      // Ups, the data is not available anymore
 *     }
 *   }
 */

// Include implementation header files
#include <ddsrouter_utils/memory/impl/OwnerPtr.hpp>

#endif /* _DDSROUTERUTILS_MEMORY_OWNERPTR_HPP_ */
