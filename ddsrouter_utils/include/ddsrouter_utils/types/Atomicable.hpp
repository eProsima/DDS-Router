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
 * @file Atomicable.hpp
 *
 * This file contains class Atomicable definition.
 */

#ifndef _DDSROUTERUTILS_TYPES_ATOMICABLE_HPP_
#define _DDSROUTERUTILS_TYPES_ATOMICABLE_HPP_

#include <mutex>
#include <shared_mutex>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Utils class to join any type value and a mutex together for commodity and readability.
 *
 * @tparam Type actual type of the object.
 * @tparam Mutex class of the mutex. Default std::mutex but could be recursive, shared, etc.
 *
 * @note as primitive types are not classes, are not allowed in this class. Use std::atomic instead.
 *
 * EXAMPLE OF USE
 *   Atomicable<Foo> f;
 *   std::unique_lock lock(f);
 *   f.foo();
 */
template <class Type, class Mutex = std::mutex>
class Atomicable : public Type, public Mutex
{
    // Nothing to add
};

/**
 * Alias to use a Atomicable object with a shared time mutex.
 *
 * EXAMPLE OF USE
 *   Atomic<Foo> f;
 *   std::shared_lock lock(f);
 *   f.foo();
 */
template <class Type>
using SharedAtomicable = Atomicable<Type, std::shared_timed_mutex>;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TYPES_ATOMICABLE_HPP_ */
