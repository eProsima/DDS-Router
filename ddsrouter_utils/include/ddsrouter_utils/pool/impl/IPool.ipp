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
 * @file IPool.ipp
 */

#ifndef __DDSROUTERUTILS_POOL_IPOOL_IMPL_IPP_
#define __DDSROUTERUTILS_POOL_IPOOL_IMPL_IPP_

#include <ddsrouter_utils/pool/LimitlessPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
class LimitlessPool;

template <typename T>
static IPool<T>* create_pool(
        PoolConfiguration configuration)
{
    // Create pool depending on the configuration
    // TODO: change it depending on the configuration
    return new LimitlessPool<T>(configuration);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __DDSROUTERUTILS_POOL_IPOOL_IMPL_IPP_ */
