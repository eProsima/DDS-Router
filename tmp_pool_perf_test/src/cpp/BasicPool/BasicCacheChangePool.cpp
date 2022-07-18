// // Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// /**
//  * @file BasicCacheChangePool.cpp
//  */

// #include <fastdds/rtps/common/CacheChange.h>

// #include <tmp_pool_perf_test/BasicPool/BasicCacheChangePool.hpp>

// namespace eprosima {
// namespace ddsrouter {
// namespace core {

// BasicCacheChangePool::BasicCacheChangePool(utils::PoolConfiguration configuration)
// {
//     if (configuration.initial_size > 0)
//     {
//         initialize_queue_(configuration.initial_size);
//     }
// }

// bool BasicCacheChangePool::reserve_cache(
//         fastrtps::rtps::CacheChange_t*& cache_change)
// {
//     return loan(cache_change);
// }

// bool BasicCacheChangePool::release_cache(
//         fastrtps::rtps::CacheChange_t* cache_change)
// {
//     return return_loan(cache_change);
// }

// fastrtps::rtps::CacheChange_t* BasicCacheChangePool::new_element_()
// {
//     return new fastrtps::rtps::CacheChange_t();
// }

// } /* namespace core */
// } /* namespace ddsrouter */
// } /* namespace eprosima */
