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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <fastdds/rtps/history/IChangePool.h>
#include <fastdds/rtps/resources/ResourceManagement.h>
#include <ddsrouter_utils/time/Timer.hpp>

#include <tmp_pool_perf_test/FastDdsChangePool/CacheChangePool.h>
#include <tmp_pool_perf_test/RouterPool/RouterCacheChangePool.hpp>
#include <tmp_pool_perf_test/RefactorFastDdsChangePool/RefactorCacheChangePool.h>

constexpr const unsigned int TEST_ITERATIONS = 10000;

enum class PoolType
{
    fastdds,
    router,
    refactor,
};

eprosima::fastrtps::rtps::IChangePool* get_new_fastdds_pool()
{
    // Configure the pool
    eprosima::fastrtps::rtps::PoolConfig config;
    config.memory_policy = eprosima::fastrtps::rtps::MemoryManagementPolicy::DYNAMIC_REUSABLE_MEMORY_MODE;
    config.payload_initial_size = 0;
    config.initial_size = 0;
    config.maximum_size = 0;

    // Create pool
    return new eprosima::fastrtps::rtps::CacheChangePool(config);
}

eprosima::fastrtps::rtps::IChangePool* get_new_router_pool()
{
    // Configure the pool
    eprosima::ddsrouter::utils::PoolConfiguration config;
    config.initial_size = 0;
    config.maximum_size = 0;
    config.batch_size = 1;

    // Create pool
    return new eprosima::ddsrouter::core::CacheChangePool(config);
}

eprosima::fastrtps::rtps::IChangePool* get_new_refactor_pool()
{
    // Configure the pool
    eprosima::fastrtps::rtps::PoolConfig config;
    config.memory_policy = eprosima::fastrtps::rtps::MemoryManagementPolicy::DYNAMIC_REUSABLE_MEMORY_MODE;
    config.payload_initial_size = 0;
    config.initial_size = 0;
    config.maximum_size = 0;

    // Create pool
    return new eprosima::fastrtps::rtps::RefactorCacheChangePool(config);
}

eprosima::fastrtps::rtps::IChangePool* get_new_pool(
    PoolType pool_type)
{
    switch (pool_type)
    {
        case PoolType::fastdds:
            return get_new_fastdds_pool();

        case PoolType::router:
            return get_new_router_pool();

        case PoolType::refactor:
            return get_new_refactor_pool();

        default:
            return nullptr;
    }
}

double performance_test(
    PoolType pool_type,
    unsigned int num_elements)
{
    double total_time = 0.0;

    for (unsigned int i = 0; i < TEST_ITERATIONS; ++i)
    {
        // Create new vector and new pool for each iteration
        std::vector<eprosima::fastrtps::rtps::CacheChange_t*> changes(num_elements);
        eprosima::fastrtps::rtps::IChangePool* pool = get_new_pool(pool_type);

        eprosima::ddsrouter::utils::Timer timer;

        for (unsigned int i = 0; i < num_elements; ++i)
        {
            pool->reserve_cache(changes[i]);
        }

        for (unsigned int i = 0; i < num_elements; ++i)
        {
            pool->release_cache(changes[i]);
        }

        total_time += timer.elapsed();

        delete pool;
    }

    return total_time / TEST_ITERATIONS;
}

void test_pool(
    std::string test_name,
    PoolType pool_type)
{
    std::vector<unsigned int> sizes = {
        // 1
        // 1, 2, 4, 16, 256
        1, 2, 4, 16, 256, 2048
    };

    for (auto size : sizes)
    {
        double result = performance_test(pool_type, size);
        std::cout << test_name << ":" << size << ": " << result << std::endl;
    }
}

TEST(ChangePoolPerformanceTest, fastdds_pool)
{
    test_pool("FastDds", PoolType::fastdds);
}

TEST(ChangePoolPerformanceTest, router_pool)
{
    test_pool("Router", PoolType::router);
}

TEST(ChangePoolPerformanceTest, refactor_pool)
{
    test_pool("Refactor", PoolType::refactor);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
