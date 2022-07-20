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
 * @file ReallocatedPool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_REALLOCATESPOOL_HPP_
#define _DDSROUTERUTILS_POOL_REALLOCATESPOOL_HPP_

#include <stack>

#include <ddsrouter_utils/pool/PerformancePool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
class ReallocatedPool : PerformancePool<T>
{
public:

    ReallocatedPool(
            const PoolConfiguration& configuration,
            const BatchPoolManager& manager)
        : manager_(manager)
        , maximum_size_(configuration.maximum_size)
        , reserved_(configuration.maximum_size)
    {
        for (size_t i = 0; i < configuration.initial_size; ++i)
        {
            T* element;
            manager_.new_element_function_(element);
            elements_.push(element);
        }
    }

    ~ReallocatedPool()
    {
        while (!elements_.empty())
        {
            manager_.delete_element_function_(elements_.top());
            elements_.pop();
        }
    }

    bool loan(
            T*& element) override
    {
        if (elements_.empty())
        {
            if (reserved_ < maximum_size_)
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
        else
        {
            element = elements_.top();
            elements_.pop();
            return true;
        }
    }

    bool return_loan(
            T* element) override
    {
        elements_.push(element);
        return true;
    }

protected:

    //! Pool configuration.
    const BatchPoolManager manager_;

    std::stack<T*> elements_;

    unsigned int maximum_size_;
    unsigned int reserved_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_POOL_REALLOCATESPOOL_HPP_ */
