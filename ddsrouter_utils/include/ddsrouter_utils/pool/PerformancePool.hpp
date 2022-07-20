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

#ifndef _DDSROUTERUTILS_POOL_PERFORMANCEPOOL_HPP_
#define _DDSROUTERUTILS_POOL_PERFORMANCEPOOL_HPP_

#include <functional>

namespace eprosima {
namespace ddsrouter {
namespace utils {

struct PoolConfiguration
{
    PoolConfiguration() = default;

    bool reusable = true;
    bool allow_reallocation = true;

    unsigned int initial_size = 0;
    unsigned int maximum_size = 0;

};

template <typename T>
struct PoolManager
{
    PoolManager() = default;

    std::function<void(T*&)> new_element_function_(
        [](T*& t){ t = new T(); });

    std::function<void(T*)> reset_element_function_(
        [](T* ){ });

    std::function<void(T*)> delete_element_function_(
        [](T* t){ delete t; });

};

template <typename T>
struct BatchPoolManager : public PoolManager<T>
{
    BatchPoolManager() = default;

    std::function<unsigned int(unsigned int)> new_batch_size_(
        [](unsigned int b){ return b; });

};

template <typename T>
static IPool<T>* create_pool(
        PoolConfiguration configuration
        PoolManager<T> manager);

template <typename T>
static IPool<T>* create_pool(
        const eprosima::fastrtps::rtps::PoolConfig& config,
        PoolManager<T> manager)
{
    PoolConfiguration configuration;
    configuration.reusable = config.reusable;
    configuration.allow_reallocation = config.allow_reallocation;
    configuration.initial_size = config.initial_size;
    configuration.maximum_size = config.maximum_size;
    return create_pool(configuration, manager);
}

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
class PerformancePool
{
public:

    virtual bool loan(
            T*& element);

    virtual bool return_loan(
            T* element);
};

/**
 * @brief Create a pool object depending on the configuration given.
 *
 * It creates the IPool specialization that better implements the given configuration.
 *
 * @tparam T Type of elements that the Pool will manage.
 * @param configuration Pool Configuration
 * @return IPool<T>* new Pool object.
 */

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/pool/impl/PerformancePool.ipp>

#endif /* _DDSROUTERUTILS_POOL_PERFORMANCEPOOL_HPP_ */
