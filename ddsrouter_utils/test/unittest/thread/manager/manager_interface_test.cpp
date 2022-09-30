// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddsrouter_utils/wait/IntWaitHandler.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_utils/math/math.hpp>
#include <ddsrouter_utils/thread/manager/IManager.hpp>
#include <ddsrouter_utils/thread/manager/SyncManager.hpp>
#include <ddsrouter_utils/thread/manager/AsyncManager.hpp>
#include <ddsrouter_utils/thread/manager/StdThreadPool.hpp>
#include <ddsrouter_utils/thread/task/OwnedTask.hpp>

using namespace eprosima::ddsrouter;

namespace test {

utils::Duration_ms DEFAULT_TIME_TEST = 20u;
unsigned int DEFAULT_TIME_REPETITIONS = 20u;
unsigned int DEFAULT_THREADS = 3u;

/**
 * @brief Function that increases \c counter in \c increase so can be checked that has been successfully done.
 *
 * @param counter Wait Handler that holds the number of increases done to the same variable
 * @param increase value to increase \c counter
 */
void test_lambda_increase_waiter(
        event::IntWaitHandler& counter,
        unsigned int increase = 1)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIME_TEST));
    for (unsigned int i = 0; i < increase; ++i)
    {
        ++counter;
    }
}

/**
 * Task type to use.
 * Using \c OwnedTask because it is the simpler and easier one.
 */
using TaskType = utils::thread::OwnedTask;

template <class Manager>
utils::thread::IManager* create_manager_interface()
{
    return new Manager();
}

template <>
utils::thread::IManager* create_manager_interface<utils::thread::StdThreadPool>()
{
    utils::thread::StdThreadPool* pool = new utils::thread::StdThreadPool(DEFAULT_THREADS, false);
    pool->start();
    return pool;
}

} /* namespace eprosima */

using namespace eprosima::ddsrouter;

template<class T>
struct ThreadManagerTest : public ::testing::Test
{};

TYPED_TEST_SUITE_P(ThreadManagerTest);

TYPED_TEST_P(ThreadManagerTest, manager_execute)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);

    // Create manager
    utils::thread::IManager* manager(
        test::create_manager_interface<TypeParam>());

    // Execute lambda increasing in 1
    {
        auto lambda = [&counter](){ test::test_lambda_increase_waiter(counter, 1); };
        test::TaskType task(lambda);
        manager->execute(std::make_unique<test::TaskType>(lambda));
    }
    // Wait for lambda to be called required times
    counter.wait_equal(1);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), 1);
    // Reset counter
    counter.set_value(0);

    // Execute lambda N times by copy increasing in 1
    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        manager->execute(
            std::make_unique<test::TaskType>(
                [&counter, i](){ test::test_lambda_increase_waiter(counter, i); }));
    }

    // Wait for lambda to be called required times
    uint32_t target_value = utils::arithmetic_progression_sum(1, 1, test::DEFAULT_TIME_REPETITIONS);
    counter.wait_equal(target_value);

    // Erase Manager
    delete manager;

    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), target_value);
}

// Register test class and test cases
REGISTER_TYPED_TEST_SUITE_P(
    ThreadManagerTest,
    manager_execute
);

// Set types used in parametrization
typedef ::testing::Types<
        utils::thread::SyncManager,
        utils::thread::AsyncManager,
        utils::thread::StdThreadPool
    > CaseTypes;

// Generate each test case for each type case
INSTANTIATE_TYPED_TEST_SUITE_P(
    ParametrizedThreadManagerTest,
    ThreadManagerTest,
    CaseTypes);


int main(
        int argc,
        char** argv)
{
    // utils::Log::SetVerbosity(utils::Log::Kind::Info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
