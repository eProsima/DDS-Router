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

#include <cmath>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/math/math.hpp>
#include <ddsrouter_utils/thread/manager/StdThreadPool.hpp>
#include <ddsrouter_utils/thread/task/OwnedTask.hpp>
#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_utils/wait/IntWaitHandler.hpp>

using namespace eprosima::ddsrouter;

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

constexpr const utils::Duration_ms DEFAULT_TIME_TEST = 200u;  // T
constexpr const utils::Duration_ms RESIDUAL_TIME_TEST = DEFAULT_TIME_TEST / 2u;  // dT

constexpr const uint32_t N_THREADS_IN_TEST = 10;  // N
constexpr const uint32_t N_EXECUTIONS_IN_TEST = 20;  // M

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

/**
 * TESTS EXPLANATION
 * These tests create a StdThreadPool and execute tasks in it.
 *
 * Tasks:
 * Tasks objects used are OwnedTask and are created in the moment to send it to execute, so they will be destroyed
 * automatically when finishing the task.
 *
 * Task function:
 * The function used waits for a time T and increases a WaitHandler value the amount of time given by parameter.
 * The WaitHandler is used so the test can wait in main thread to the expected value.
 *
 * Parameteres:
 * Two parameters are used within the tests:
 * @param n_threads Number of threads
 * @param m_tasks Number of repetitions (#tasks added to pool)
 *
 * @warning if \c m_tasks is not dividible by \c n_threads the test may not work as expected because of
 * non exact division solution.
 */
void test_thread_pool_with_parameters(
        unsigned int n_threads,
        unsigned int m_tasks)
{
    // Create thread_pool
    thread::StdThreadPool thread_pool(n_threads, false);
    thread_pool.start();

    // Create timer to know the task has been executed in the time expected
    utils::Timer timer;

    // Counter Wait Handler to wait for the task to be executed and check the final value
    event::IntWaitHandler waiter(0);

    // Emit N tasks n times
    for (uint32_t i = 1; i <= m_tasks; ++i)
    {
        thread_pool.execute(
            std::make_unique<test::TaskType>(
                [&waiter, i] () { test::test_lambda_increase_waiter(waiter, i); }
            )
        );
    }

    // Wait for counter value to be greater than 0 (so 1 task is being executed)
    uint32_t target_value = utils::arithmetic_progression_sum(1, 1, m_tasks);
    waiter.wait_greater_equal_than(target_value);

    auto time_elapsed = timer.elapsed();

    // Check that the task has been executed in more than the time expected for lambda and less than expected
    // time and residual; and that function has been called exactly once
    double lower_time_expected = test::DEFAULT_TIME_TEST * std::floor(m_tasks / n_threads);
    double higher_time_expected = test::DEFAULT_TIME_TEST * std::ceil(m_tasks / n_threads) + test::RESIDUAL_TIME_TEST;

    ASSERT_GE(time_elapsed, lower_time_expected);
    ASSERT_LE(time_elapsed, higher_time_expected);
    ASSERT_EQ(waiter.get_value(), target_value);

    // Thread Pool is destroyed automatically and without errors
}

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace utils;

/**
 * Emit 1 tasks to a ThreadPool with 1 thread.
 * Check that time elapsed is > T
 */
TEST(StdThreadPoolTest, pool_1_threads_1_tasks)
{
    test::test_thread_pool_with_parameters(1, 1);
}

/**
 * Emit M tasks to a ThreadPool with 1 thread.
 */
TEST(StdThreadPoolTest, pool_1_threads_M_tasks)
{
    test::test_thread_pool_with_parameters(1, test::N_EXECUTIONS_IN_TEST);
}

/**
 * Emit N tasks to a ThreadPool with N threads.
 */
TEST(StdThreadPoolTest, pool_N_threads_N_tasks)
{
    test::test_thread_pool_with_parameters(test::N_THREADS_IN_TEST, test::N_THREADS_IN_TEST);
}

/**
 * Emit M*N tasks to a ThreadPool with N threads.
 */
TEST(StdThreadPoolTest, pool_N_threads_NM_tasks)
{
    test::test_thread_pool_with_parameters(
        test::N_THREADS_IN_TEST,
        test::N_THREADS_IN_TEST * test::N_EXECUTIONS_IN_TEST);
}

int main(
        int argc,
        char** argv)
{
    // eprosima::ddsxrouter::utils::Log::SetVerbosity(utils::Log::Kind::Info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
