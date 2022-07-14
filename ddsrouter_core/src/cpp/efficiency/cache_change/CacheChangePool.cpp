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
 * @file CacheChangePool.cpp
 */

#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <efficiency/cache_change/CacheChangePool.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

CacheChangePool::CacheChangePool(
        CacheChangePoolConfiguration memory_policy)
    : free_values_(memory_policy.maximum_size)
    , reserved_(memory_policy.initial_size)
    , index_first_free_available_(memory_policy.initial_size)
    , configuration_(memory_policy)
{
    // Create the initial values
    for (unsigned int i = 0; i < memory_policy.initial_size; ++i)
    {
        free_values_[i] = new types::RouterCacheChange();
    }
}

CacheChangePool::~CacheChangePool()
{
    // Check that every element has been released
    if (reserved_ != index_first_free_available_)
    {
        utils::InconsistencyException("More Elements released than reserved.");
    }

    // Delete the values
    for (unsigned int i = 0; i < reserved_; ++i)
    {
        delete free_values_[i];
    }
}

bool CacheChangePool::reserve_cache(
        fastrtps::rtps::CacheChange_t*& cache_change)
{
    if (index_first_free_available_ == 0)
    {
        // It requires to allocate a new value
        if (reserved_ == configuration_.maximum_size)
        {
            // The pool is full
            return false;
        }
        else
        {
            // Allocate a new value
            ++reserved_;
            cache_change = new types::RouterCacheChange();
        }
    }
    else
    {
        // It uses an existing already allocated value
        cache_change = free_values_[index_first_free_available_ - 1];
        --index_first_free_available_;
    }

    return true;
}

//! Override \c IChangePool \c release_cache to release the cache change to the pool.
bool CacheChangePool::release_cache(
        fastrtps::rtps::CacheChange_t* cache_change)
{
    // This only could happen if more cache changes are released than reserved.
    if(index_first_free_available_ == free_values_.size())
    {
        utils::tsnh(STR_ENTRY << "CacheChangePool::release_cache: More cache changes are released than reserved.");
    }

    // Add it to vector
    free_values_[index_first_free_available_] = cache_change;
    ++index_first_free_available_;

    return true;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
