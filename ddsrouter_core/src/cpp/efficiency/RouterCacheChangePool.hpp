// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RouterCacheChangePool.h
 */

#ifndef DDSROUTER_CACHECHANGEPOOL_H_
#define DDSROUTER_CACHECHANGEPOOL_H_

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/history/IChangePool.h>
#include <ddsrouter_core/types/dds/RouterCacheChange.hpp>
#include <fastdds/rtps/history/IChangePool.h>
#include <fastdds/rtps/resources/ResourceManagement.h>

#include <fastdds/rtps/history/PoolConfig.h>

#include <vector>
#include <algorithm>
#include <cstdint>
#include <cstddef>

namespace eprosima {
namespace fastrtps {
namespace rtps {

/**
 * Class RouterCacheChangePool, used by the HistoryCache to pre-reserve a number of CacheChange_t to avoid dynamically
 * reserving memory in the middle of execution loops.
 * @ingroup COMMON_MODULE
 */
class RouterCacheChangePool : public IChangePool // public ChangeChangePool TODO anton see if we can reuse some code from it
{
public:

    virtual ~RouterCacheChangePool();

    /**
     * Construct and initialize a RouterCacheChangePool.
     *
     * @param config   Pool configuration (member @c payload_initial_size is not being used).
     * @param f        Functor to be called on all preallocated elements.
     */
    template<class UnaryFunction>
    RouterCacheChangePool(
            const PoolConfig& config,
            UnaryFunction f)
    {
        init(config);
        std::for_each(all_caches_.begin(), all_caches_.end(), f);
    }

    /**
     * Construct and initialize a RouterCacheChangePool.
     *
     * @param config   Pool configuration (member @c payload_initial_size is not being used).
     */
    RouterCacheChangePool(
            const PoolConfig& config)
    {
        init(config);
    }

    bool reserve_cache(
            CacheChange_t*& cache_change) override;

    bool release_cache(
            CacheChange_t* cache_change) override;

    //!Get the size of the cache vector; all of them (reserved and not reserved).
    size_t get_allCachesSize()
    {
        return all_caches_.size();
    }

    //!Get the number of free caches.
    size_t get_freeCachesSize()
    {
        return free_caches_.size();
    }

protected:

    /**
     * Construct a RouterCacheChangePool without initialization.
     */
    RouterCacheChangePool() = default;

    void init(
            const PoolConfig& config);

    virtual CacheChange_t* create_change() const
    {
        return new RouterCacheChange_t();
    }

    virtual void destroy_change(
            CacheChange_t* change) const
    {
        delete change;
    }

private:

    uint32_t current_pool_size_ = 0;
    uint32_t max_pool_size_ = 0;
    MemoryManagementPolicy_t memory_mode_ = MemoryManagementPolicy_t::DYNAMIC_RESERVE_MEMORY_MODE;

    std::vector<RouterCacheChange_t*> free_caches_;
    std::vector<RouterCacheChange_t*> all_caches_;

    bool allocateGroup(
            uint32_t num_caches);

    RouterCacheChange_t* allocateSingle();

    //! Returns a RouterCacheChange to the free caches pool
    void return_cache_to_pool(
            RouterCacheChange_t* ch);

};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif /* DDSROUTER_CACHECHANGEPOOL_H_ */

