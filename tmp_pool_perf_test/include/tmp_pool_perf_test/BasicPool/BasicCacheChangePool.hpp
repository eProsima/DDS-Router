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
 * @file BasicCacheChangePool.hpp
 */

#ifndef __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOL_HPP_
#define __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOL_HPP_

#include <fastdds/rtps/history/IChangePool.h>

#include <ddsrouter_utils/pool/LimitlessPool.hpp>
#include <ddsrouter_utils/pool/IPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * @brief This class implements a pool of CacheChange objects specialized as RouterCacheChanges.
 *
 * It reuses the LimitlessPool implementation, what allow to create a limitless reusable pool.
 *
 * TODO: implement this class as an IPool (or having an internal pool), without being force to be limitless.
 */
class BasicCacheChangePool : public fastrtps::rtps::IChangePool
{
public:

    /**
     * @brief Construct a new Cache Change Pool object from a Pool Configuration
     *
     * @param configuration pool configuration
     *
     * @warning max size will not be used as it is Limitedless
     */
    BasicCacheChangePool(utils::PoolConfiguration configuration);

    //! Call LimitlessPool::reserve
    virtual bool reserve_cache(
            fastrtps::rtps::CacheChange_t*& cache_change) override;

    //! Call LimitlessPool::release
    virtual bool release_cache(
            fastrtps::rtps::CacheChange_t* cache_change) override;

protected:

    //! Override the LimitlessPool::create_element method to create a RouterCacheChange object.
    virtual fastrtps::rtps::CacheChange_t* new_element_();

};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOL_HPP_ */
