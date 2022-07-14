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
 * @file PoolConfiguration.hpp
 */

#ifndef __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOLCONFIGURATION_HPP_
#define __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOLCONFIGURATION_HPP_

#include <ddsrouter_utils/pool/PoolConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

struct CacheChangePoolConfiguration
{
    CacheChangePoolConfiguration(
            utils::MemoryManagementPolicy memory_policy = utils::MemoryManagementPolicy::DYNAMIC_RESERVE_MEMORY_MODE,
            unsigned int initial_size = 0,
            unsigned int maximum_size = 0,
            unsigned int batch_size = 1) noexcept;

    //! Initial number of elements when preallocating data.
    unsigned int initial_size;
    unsigned int maximum_size;
    unsigned int batch_size;

    utils::MemoryManagementPolicy memory_policy;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_EFFICIENCY_CACHECHANGE_CACHACHANGEPOOLCONFIGURATION_HPP_ */
