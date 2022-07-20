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
 * @file FixedPool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_FIXEDPOOL_HPP_
#define _DDSROUTERUTILS_POOL_FIXEDPOOL_HPP_

#include <vector>

#include <ddsrouter_utils/pool/PerformancePool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
class FixedPool : PerformancePool<T>
{
public:

    FixedPool(
            const PoolConfiguration& configuration,
            const PoolManager& manager)
        : manager_(manager)
        , elements_(configuration.maximum_size)
        , first_free_(configuration.maximum_size)
    {
        std::for_each(elements_.begin(), elements_.end(), manager_.new_element_function_);
    }

    ~FixedPool()
    {
        std::for_each(elements_.begin(), elements_.end(), manager_.delete_element_function_);
    }

    bool loan(
            T*& element) override
    {
        if (first_free_ == 0)
        {
            return false;
        }
        else
        {
            first_free_--;
            element = elements_[first_free_];
            return true;
        }
    }

    bool return_loan(
            T* element) override
    {
        manager_.reset_element_function_(element);
        elements_[first_free_] = element;
        first_free_++;
        return true;
    }

protected:

    //! Pool configuration.
    const PoolManager manager_;

    std::vector<T*> elements_;

    unsigned int first_free_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_POOL_FIXEDPOOL_HPP_ */
