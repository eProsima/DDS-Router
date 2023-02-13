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

#pragma once

#include <fastdds/rtps/history/IChangePool.h>

#include <cpp_utils/pool/UnboundedPool.hpp>
#include <cpp_utils/pool/IPool.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * @brief This class implements a pool of CacheChange objects specialized as RouterCacheChanges.
 *
 * It reuses the UnboundedPool implementation, what allow to create an unbounded reusable pool.
 *
 * TODO: implement this class as an IPool (or having an internal pool), without being force to be unbounded.
 */
class CacheChangePool : public fastrtps::rtps::IChangePool, public utils::UnboundedPool<fastrtps::rtps::CacheChange_t>
{
public:

    /**
     * @brief Construct a new Cache Change Pool object from a Pool Configuration
     *
     * @param configuration pool configuration
     *
     * @warning max size will not be used as it is Unbounded
     */
    CacheChangePool(
            utils::PoolConfiguration configuration);

    //! Call UnboundedPool::reserve
    virtual bool reserve_cache(
            fastrtps::rtps::CacheChange_t*& cache_change) override;

    //! Call UnboundedPool::release
    virtual bool release_cache(
            fastrtps::rtps::CacheChange_t* cache_change) override;

protected:

    //! Override the UnboundedPool::create_element method to create a RouterCacheChange object.
    virtual fastrtps::rtps::CacheChange_t* new_element_() override;

};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
