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
 * @file UnboundedPool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_UNBOUNDEDPOOL_HPP_
#define _DDSROUTERUTILS_POOL_UNBOUNDEDPOOL_HPP_

#include <vector>

#include <ddsrouter_utils/pool/IPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief This class implements a generic not-thread-safe reusable Pool without size limit.
 *
 * ATTRIBUTES:
 * - Reuse freed elements without allocation.
 * - Not thread safe
 * - No pool size limit
 *
 * This implementation uses an internal vector where new elements are stored.
 * Whenever it runs out of free (not loaned) elements, it allocates more following configuration batch.
 * Whenever an element is returned, it turns back to the vector to be reused.
 *
 * @warning This class is not thread safe.
 *
 * @tparam T Type of the elements in the pool.
 */
template <typename T>
class UnboundedPool : IPool<T>
{
public:

    /**
     * @brief Create a new UnboundedPool object by a Pool Configuration given.
     *
     * It initializes the internal vector with the given initial size, using \c new_element_ .
     *
     * @note Whenever this class is inherited, the constructor must call \c initialize_vector_ ,
     * as this class cannot because \c new_element_ must be overriden by the child class.
     *
     * @param configuration Pool Configuration
     *
     * @throw InitializationException if the pool configuration is not correct.
     */
    UnboundedPool(
            PoolConfiguration configuration);

    /**
     * @brief Destroy the Limitless Pool object and all its reserved elements.
     */
    ~UnboundedPool();

    /**
     * @brief Override IPool::loan
     *
     * If there are elements not in use in the vector, it returns one of them in \c element .
     * Otherwise, it allocates more elements and takes the next free one.
     */
    virtual bool loan(
            T*& element) override;

    /**
     * @brief Override IPool::return_loan
     *
     * Return the element to the vector.
     *
     * @throw InconsistencyException if there have been more released than reserved calls.
     */
    virtual bool return_loan(
            T* element) override;

protected:

    /**
     * @brief Initialize the internal vector with the given initial size in configuration.
     *
     * This method must be called in every final child class constructor.
     */
    virtual void initialize_vector_();

    /**
     * @brief Augment the internal vector with \c batch new elements using \c new_element_ function for each of them.
     */
    void augment_free_values_();

    /**
     * @brief vector where elements are stored.
     *
     * Each element of the vector has been initialized with \c new_element_
     * or returned to the pool after calling \c reset_element_ .
     * The elements are added and consumed at the back.
     */
    std::vector<T*> elements_;

    /**
     * @brief Total number of elements reserved by the pool.
     *
     * This is only a debugging variable that may be deleted for performance reasons.
     */
    unsigned int reserved_;

    //! Pool configuration.
    const PoolConfiguration configuration_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/pool/impl/UnboundedPool.ipp>

#endif /* _DDSROUTERUTILS_POOL_UNBOUNDEDPOOL_HPP_ */
