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
 * @file AtomicableValue.hpp
 *
 * This file contains class Atomicable definition.
 */

#ifndef _DDSROUTERUTILS_TYPES_ATOMICABLEVALUE_HPP_
#define _DDSROUTERUTILS_TYPES_ATOMICABLEVALUE_HPP_

#include <ddsrouter_utils/types/Atomicable.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief TODO
 *
 * @tparam T
 */
template <class T>
class AtomicableValue
{
public:

    AtomicableValue(const T& value);

    AtomicableValue(T&& value);

    void get(T& value_to_set) const noexcept;

    void set(const T& new_value) const;

    void set(T&& new_value) const;

protected:

    SharedAtomicable<T> value_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/types/impl/AtomicableValue.ipp>

#endif /* _DDSROUTERUTILS_TYPES_ATOMICABLEVALUE_HPP_ */
