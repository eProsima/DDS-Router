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
 * @file IPool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_IPOOL_HPP_
#define _DDSROUTERUTILS_POOL_IPOOL_HPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Memory Pool interface.
 *
 * A Pool of elements manage the memory allocation of the elements.
 * Whenever a new element wants to be used, \c loan is called.
 * Every element must be returned to the pool when it is not used anymore by \c return_loan .
 *
 * This Pools are meant to efficiently manage allocations and memory, so loan and return_loan could not reserve
 * or release any memory, but to manage to reuse, reallocate, move element, etc.
 *
 * @tparam T Type of elements that the Pool will manage.
 */
template <typename T>
class IPool
{
public:

    /**
     * @brief Get a new element from the pool.
     *
     * @param [out] element reference to the ptr that will be filled with the new element.
     * @return true if loan was successful.
     * @return false otherwise.
     */
    virtual bool loan(
            T*& element) = 0;

    /**
     * @brief Return an element to the pool.
     *
     * @param [in] element reference to the element belonging to this Pool that is being returned.
     * @return true if release was successful.
     * @return false otherwise.
     *
     * @warning After calling this method, \c element is not valid anymore.
     *
     * @pre \c element must have been reserved by this same Pool.
     */
    virtual bool return_loan(
            T* element) = 0;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/pool/impl/IPool.ipp>

#endif /* _DDSROUTERUTILS_POOL_IPOOL_HPP_ */
