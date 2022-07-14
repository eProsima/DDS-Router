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
 * @file LimitlessPool.ipp
 */

#ifndef __DDSROUTERUTILS_POOL_LIMITLESSPOOL_IMPL_IPP_
#define __DDSROUTERUTILS_POOL_LIMITLESSPOOL_IMPL_IPP_

#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
LimitlessPool<T>::LimitlessPool(
        PoolConfiguration configuration)
    : free_values_(configuration.initial_size)
    , reserved_(0)
    , index_first_void_(0)
    , configuration_(configuration)
{
    // Call initialize_vector_ in every child constructor
}

template <typename T>
LimitlessPool<T>::~LimitlessPool()
{
    // Check that every element has been released
    if (reserved_ != index_first_void_)
    {
        utils::InconsistencyException("More Elements released than reserved.");
    }

    // Delete the values
    for (unsigned int i = 0; i < reserved_; ++i)
    {
        delete free_values_[i];
    }
}

template <typename T>
bool LimitlessPool<T>::reserve(
        T*& element)
{
    if (index_first_void_ == 0)
    {
        // It requires to allocate new values
        augment_free_values_();
    }

    // There are already free values available
    // It uses an existing already allocated value
    element = free_values_[--index_first_void_];

    return true;
}

template <typename T>
bool LimitlessPool<T>::release(
        T* cache_change)
{
    // This only could happen if more elements are released than reserved.
    if(index_first_void_ == free_values_.size())
    {
        utils::tsnh(STR_ENTRY << "release_cache: More elements are released than reserved.");
    }

    // Add it to vector
    free_values_[index_first_void_++] = cache_change;

    return true;
}

template <typename T>
void LimitlessPool<T>::augment_free_values_()
{
    for (unsigned int i = 0; i < this->configuration_.batch_size; ++i)
    {
        this->free_values_.push_back(this->new_element_());
    }
    index_first_void_ += this->configuration_.batch_size;
}

template <typename T>
void LimitlessPool<T>::initialize_vector_()
{
    for (unsigned int i = 0; i < this->configuration_.initial_size; ++i)
    {
        free_values_[i] = new_element_();
    }
    index_first_void_ = this->configuration_.initial_size;
    reserved_ = this->configuration_.initial_size;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __DDSROUTERUTILS_POOL_LIMITLESSPOOL_IMPL_IPP_ */
