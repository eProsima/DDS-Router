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

#include <ddsrouter_utils/thread_pool/pool/SlotThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

eprosima::ddsrouter::utils::Duration_ms DEFAULT_TIME_TEST = 200u;
eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME_TEST = DEFAULT_TIME_TEST / 2u;

uint32_t N_THREADS_IN_TEST = 10;
uint32_t N_EXECUTIONS_IN_TEST = 5;

void test_lambda_increase_waiter(eprosima::ddsrouter::event::IntWaitHandler& counter, unsigned int increase = 1)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIME_TEST));

    for (unsigned int i = 0; i < increase; ++i)
    {
        ++counter;
    }
}

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::utils;

/**
 * Emit N tasks to a ThreadPool with one thread by storing slot.
 */
TEST(slot_thread_pool_test, pool_one_thread_one_slot)
{
    // Create thread_pool
    SlotThreadPool thread_pool(1);

    // Counter Wait Handler to wait for the task to be executed and check the final value
    eprosima::ddsrouter::event::IntWaitHandler waiter(0);

    // Create slot
    TaskId task_id(27);
    thread_pool.slot(
        task_id,
        [&waiter]
        ()
        {
            test::test_lambda_increase_waiter(waiter);
        }
    );

    // Emit task n times
    for (uint32_t i = 0; i < test::N_EXECUTIONS_IN_TEST; ++i)
    {
        thread_pool.emit(task_id);
    }

    // Wait for counter value to be greater than 0 (so 1 task is being executed)
    waiter.wait_greater_equal_than(test::N_EXECUTIONS_IN_TEST);

    ASSERT_EQ(waiter.get_value(), test::N_EXECUTIONS_IN_TEST);
}

/**
 * Emit N tasks to a ThreadPool with one thread by storing N slots.
 */
TEST(slot_thread_pool_test, pool_one_thread_n_slots)
{

    // Create thread_pool
    SlotThreadPool thread_pool(1);

    // Counter Wait Handler to wait for the task to be executed and check the final value
    eprosima::ddsrouter::event::IntWaitHandler waiter(0);
    // Create timer to know the task has been executed in the time expected
    eprosima::ddsrouter::utils::Timer timer;

    // Create slot
    for (uint32_t i = 0; i < test::N_EXECUTIONS_IN_TEST; ++i)
    {
        TaskId task_id(i);
        thread_pool.slot(
            task_id,
            [&waiter, &i]
            ()
            {
                test::test_lambda_increase_waiter(waiter, i);
            }
        );
    }

    // Emit every task 1 time
    for (uint32_t i = 0; i < test::N_EXECUTIONS_IN_TEST; ++i)
    {
        thread_pool.emit(TaskId(i));
    }

    // Wait for counter value to be M being M = N*(N+1)/2 that is the increase value that should be achieved
    waiter.wait_greater_equal_than((test::N_EXECUTIONS_IN_TEST * (test::N_EXECUTIONS_IN_TEST + 1))/2);

    ASSERT_EQ(waiter.get_value(), (test::N_EXECUTIONS_IN_TEST * (test::N_EXECUTIONS_IN_TEST + 1))/2);
}

/**
 * Emit N*T tasks to a ThreadPool with T threads by storing 1 slot.
 */
TEST(slot_thread_pool_test, pool_n_threads_one_slot)
{

    // Create thread_pool
    SlotThreadPool thread_pool(test::N_THREADS_IN_TEST);

    // Counter Wait Handler to wait for the task to be executed and check the final value
    eprosima::ddsrouter::event::IntWaitHandler waiter(0);
    // Create timer to know the task has been executed in the time expected
    eprosima::ddsrouter::utils::Timer timer;

    // Create slot
    TaskId task_id(27);
    thread_pool.slot(
        task_id,
        [&waiter]
        ()
        {
            test::test_lambda_increase_waiter(waiter);
        }
    );

    // Emit task n times
    for (uint32_t i = 0; i < test::N_EXECUTIONS_IN_TEST * test::N_THREADS_IN_TEST; ++i)
    {
        thread_pool.emit(task_id);
    }

    // Wait for counter value to be greater than 0 (so 1 task is being executed)
    waiter.wait_greater_equal_than(test::N_EXECUTIONS_IN_TEST * test::N_THREADS_IN_TEST);

    auto time_elapsed = timer.elapsed();

    // Check that the task has been executed in more than waiting time and less than waiting time + residual time
    // and that function has been called exactly once
    ASSERT_GE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_EXECUTIONS_IN_TEST);
    ASSERT_LE(time_elapsed, test::DEFAULT_TIME_TEST * test::N_EXECUTIONS_IN_TEST + test::RESIDUAL_TIME_TEST);
    ASSERT_EQ(waiter.get_value(), test::N_EXECUTIONS_IN_TEST * test::N_THREADS_IN_TEST);
}

int main(
        int argc,
        char** argv)
{
    // eprosima::ddsxrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
