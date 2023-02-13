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

#include <efficiency/cache_change/CacheChangePool.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

CacheChangePool::CacheChangePool(
        utils::PoolConfiguration configuration)
    : utils::UnboundedPool<fastrtps::rtps::CacheChange_t>(configuration)
{
    initialize_vector_();
}

bool CacheChangePool::reserve_cache(
        fastrtps::rtps::CacheChange_t*& cache_change)
{
    return loan(cache_change);
}

bool CacheChangePool::release_cache(
        fastrtps::rtps::CacheChange_t* cache_change)
{
    return return_loan(cache_change);
}

fastrtps::rtps::CacheChange_t* CacheChangePool::new_element_()
{
    return new types::RouterCacheChange();
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
