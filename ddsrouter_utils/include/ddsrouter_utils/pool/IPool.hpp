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

#ifndef _DDSROUTERUTILS_POOL_IPOOL_HPP_
#define _DDSROUTERUTILS_POOL_IPOOL_HPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {

// TODO: reorder these classes
struct PoolConfiguration
{
    PoolConfiguration() = default;

    PoolConfiguration(
            unsigned int initial_size,
            unsigned int maximum_size,
            unsigned int batch_size) noexcept;

    //! Initial number of elements when preallocating data.
    unsigned int initial_size = 0;
    unsigned int maximum_size = 0;
    unsigned int batch_size = 1;
};

template <typename T>
class IPool
{
public:

    virtual bool reserve(
            T*& element) = 0;

    virtual bool release(
            T* element) = 0;

protected:

    virtual T* new_element_() = 0;
};

template <typename T>
static IPool<T>* create_pool(
        PoolConfiguration configuration);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/pool/impl/IPool.ipp>

#endif /* _DDSROUTERUTILS_POOL_IPOOL_HPP_ */
