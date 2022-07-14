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
 * @file CacheChangePoolConfiguration.cpp
 */

#include <efficiency/cache_change/CacheChangePoolConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

CacheChangePoolConfiguration::CacheChangePoolConfiguration(
        utils::MemoryManagementPolicy memory_policy /* = utils::MemoryManagementPolicy::DYNAMIC_RESERVE_MEMORY_MODE */,
        unsigned int initial_size /* = 0 */,
        unsigned int maximum_size /* = 0 */,
        unsigned int batch_size /* = 1 */ ) noexcept
    : memory_policy(memory_policy)
    , initial_size(initial_size)
    , maximum_size(maximum_size)
    , batch_size(batch_size)
{
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
