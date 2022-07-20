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
 * @file NonReusablePool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_NONREUSABLEPOOL_HPP_
#define _DDSROUTERUTILS_POOL_NONREUSABLEPOOL_HPP_

#include <ddsrouter_utils/pool/PerformancePool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
class NonReusablePool : PerformancePool<T>
{
public:

    NonReusablePool(
            const PoolConfiguration& configuration,
            const PoolManager& manager)
        : manager_(manager)
        , maximum_size_(configuration.maximum_size)
        , reserved_(0)
    {
    }

    bool loan(
            T*& element) override
    {
        if (reserved_ < configuration_.maximum_size)
        {
            element = manager_.new_element_function_();
            reserved_++;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool return_loan(
            T* element) override
    {
        manager_.delete_element_function_(element);
        reserved_--;
        return true;
    }

protected:

    //! Pool configuration.
    const PoolManager manager_;

    unsigned int maximum_size_;
    unsigned int reserved_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_POOL_NONREUSABLEPOOL_HPP_ */
