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
 * @file LimitlessPool.hpp
 */

#ifndef _DDSROUTERUTILS_POOL_LIMITLESSPOOL_HPP_
#define _DDSROUTERUTILS_POOL_LIMITLESSPOOL_HPP_

#include <vector>

#include <ddsrouter_utils/pool/IPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
class LimitlessPool : IPool<T>
{
public:

    LimitlessPool(
            PoolConfiguration configuration);

    ~LimitlessPool();

    virtual bool reserve(
            T*& element) override;

    virtual bool release(
            T* element) override;

protected:

    virtual void initialize_vector_();

    virtual T* new_element_() override = 0;

    void augment_free_values_();

    std::vector<T*> free_values_;

    unsigned int reserved_;

    unsigned int index_first_void_;

    const PoolConfiguration configuration_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/pool/impl/LimitlessPool.ipp>

#endif /* _DDSROUTERUTILS_POOL_LIMITLESSPOOL_HPP_ */
