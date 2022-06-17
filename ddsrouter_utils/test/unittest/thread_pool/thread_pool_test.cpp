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

#include <ddsrouter_utils/thread_pool/pool/ThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

eprosima::ddsrouter::utils::Duration_ms DEFAULT_TIME_TEST = 200u;
eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME_TEST = DEFAULT_TIME_TEST / 2u;

uint32_t N_THREADS_IN_TEST = 3;

void test_lambda_increase_waiter(eprosima::ddsrouter::event::IntWaitHandler& counter)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIME_TEST));
    ++counter;
}

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::utils;

/**
 * Create ThreadPool
 *
 * CASES:
 * - 1 thread
 * - N threads
 * - N*N threads
 */
TEST(thread_pool_test, pool_creation)
{
    // 1 thread
    {
        ThreadPool thread_pool(1);
    }

    // N thread
    {
        ThreadPool thread_pool(test::N_THREADS_IN_TEST);
    }

    // N*N thread
    {
        ThreadPool thread_pool(test::N_THREADS_IN_TEST * test::N_THREADS_IN_TEST);
    }
}

/**
 * Emit one task to a ThreadPool with one thread.
 *
 * CASES:
 * - create task in situ
 * - move task
 * - copy task
 */
TEST(thread_pool_test, pool_one_thread_emit_one)
{
    // create task in situ
    {
        // Create thread_pool
        ThreadPool thread_pool(1);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        // Emit task by movement so it is executed in other thread
        thread_pool.emit(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(0);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), 1);
    }

    // move task
    {
        // Create thread_pool
        ThreadPool thread_pool(1);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create task
        Task task(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        // Emit task by movement so it is executed in other thread
        thread_pool.emit(
            std::move(task));

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(0);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), 1);
    }

    // copy task
    {
        // Create thread_pool
        ThreadPool thread_pool(1);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create task
        Task task(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        // Emit task by copy so it is executed in other thread
        thread_pool.emit(
            task);

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(0);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), 1);
    }
}

/**
 * Emit multiple tasks to a ThreadPool with one thread.
 *
 * STEPS:
 * - send N tasks
 * - wait for N tasks to be finished
 * - wait for test time and check that no more tasks are being executed
 * - send N tasks again
 */
TEST(thread_pool_test, pool_one_thread_emit_multiple)
{
    // Create thread_pool
    ThreadPool thread_pool(1);

    // Create task
    eprosima::ddsrouter::event::IntWaitHandler waiter(0);
    Task task(
        [&waiter]
        ()
        {
            test::test_lambda_increase_waiter(waiter);
        });

    // Create Timer
    eprosima::ddsrouter::utils::Timer timer;

    //! send N tasks
    for(uint32_t i = 0; i < test::N_THREADS_IN_TEST; ++i)
    {
        thread_pool.emit(
            task);
    }

    //! wait for N tasks to be finished
    // Wait for counter value to be greater than 0 (so 1 task is being executed)
    waiter.wait_greater_than(test::N_THREADS_IN_TEST - 1);

    auto time_elapsed = timer.elapsed();

    // Check that the task has been executed in more than waiting time and less than waiting time + residual time
    // and that function has been called exactly once
    ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST);
    ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST + test::RESIDUAL_TIME_TEST);
    ASSERT_EQ(waiter.get_value(), test::N_THREADS_IN_TEST);

    //! wait for test time and check that no more tasks are being executed
    std::this_thread::sleep_for(std::chrono::milliseconds(test::DEFAULT_TIME_TEST * 2));

    // Check that counter has not increased
    ASSERT_EQ(waiter.get_value(), test::N_THREADS_IN_TEST);

    //! send N tasks again
    waiter.set_value(0);
    timer.reset();
    for(uint32_t i = 0; i < test::N_THREADS_IN_TEST; ++i)
    {
        thread_pool.emit(
            task);
    }

    //! wait for N tasks to be finished
    // Wait for counter value to be greater than 0 (so 1 task is being executed)
    waiter.wait_greater_than(test::N_THREADS_IN_TEST - 1);

    time_elapsed = timer.elapsed();

    // Check that the task has been executed in more than waiting time and less than waiting time + residual time
    // and that function has been called exactly once
    ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST);
    ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST + test::RESIDUAL_TIME_TEST);
    ASSERT_EQ(waiter.get_value(), test::N_THREADS_IN_TEST);
}

/**
 * Emit tasks to a ThreadPool with N threads.
 * Check by elapsed time whether the function has taken place in parallel
 *
 * CASES:
 * - emit 1 task per thread
 * - emit less tasks than threads (N-1 tasks)
 * - emit N tasks per thread
 */
TEST(thread_pool_test, pool_n_threads_emit)
{
    // emit 1 task per thread
    {
        // Create thread_pool
        ThreadPool thread_pool(test::N_THREADS_IN_TEST);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create task
        Task task(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        // Emit task by copy so it is executed in other thread
        for(uint32_t i = 0; i < test::N_THREADS_IN_TEST; ++i)
        {
            thread_pool.emit(
                task);
        }

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(test::N_THREADS_IN_TEST - 1);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), test::N_THREADS_IN_TEST);
    }

    // emit less tasks than threads (1 task)
    {
        // Create thread_pool
        ThreadPool thread_pool(test::N_THREADS_IN_TEST);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create task
        Task task(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        // Emit task by copy so it is executed in other thread
        for(uint32_t i = 0; i < test::N_THREADS_IN_TEST-1; ++i)
        {
            thread_pool.emit(
                task);
        }

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(test::N_THREADS_IN_TEST - 2);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), test::N_THREADS_IN_TEST - 1);
    }

    // emit N tasks per thread
    {
        // Create thread_pool
        ThreadPool thread_pool(test::N_THREADS_IN_TEST);

        // Counter Wait Handler to wait for the task to be executed and check the final value
        eprosima::ddsrouter::event::IntWaitHandler waiter(0);
        // Create task
        Task task(
            [&waiter]
            ()
            {
                test::test_lambda_increase_waiter(waiter);
            });

        // Create timer to know the task has been executed in the time expected
        eprosima::ddsrouter::utils::Timer timer;

        uint32_t n_tasks = test::N_THREADS_IN_TEST * test::N_THREADS_IN_TEST;

        // Emit task by copy so it is executed in other thread
        for(uint32_t i = 0; i < n_tasks; ++i)
        {
            thread_pool.emit(
                task);
        }

        // Wait for counter value to be greater than 0 (so 1 task is being executed)
        waiter.wait_greater_than(n_tasks - 1);

        auto time_elapsed = timer.elapsed();

        // Check that the task has been executed in more than waiting time and less than waiting time + residual time
        // and that function has been called exactly once
        ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST);
        ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_THREADS_IN_TEST + test::RESIDUAL_TIME_TEST);
        ASSERT_EQ(waiter.get_value(), n_tasks);
    }
}

int main(
        int argc,
        char** argv)
{
    // eprosima::ddsrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
