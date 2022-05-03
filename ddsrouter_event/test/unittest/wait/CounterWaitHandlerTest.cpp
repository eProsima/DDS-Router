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

#include <iostream>
#include <memory>
#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <TestLogHandler.hpp>

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {
namespace test {

eprosima::ddsrouter::utils::Duration_ms DEFAULT_TIME_TEST = 1000u;
eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME_TEST = 10u;
eprosima::ddsrouter::utils::Duration_ms LONG_TIME_TEST = 5000u;

} /* namespace test */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::event;

/**
 * \c Create a CounterWaitHandler and check its internal values with different numbers
 *
 * Change the internal values and check if the values are changed
 */
TEST(CounterWaitHandlerTest, value_handler)
{
    std::vector<CounterType> counter_values = {0, 1, 12345, -1, -99999};

    for (CounterType i : counter_values)
    {
        CounterWaitHandler waiter(i);

        ASSERT_EQ(waiter.get_value(), i);

        // Check operator++
        ++waiter;
        ASSERT_EQ(waiter.get_value(), i + 1);

        // Check operator--
        --waiter;
        ASSERT_EQ(waiter.get_value(), i);

        // Check set value
        waiter.set_value(2);
        ASSERT_EQ(waiter.get_value(), 2);
    }
}

/**
 * Test wait for a specific value.
 *
 * Wait for 1 and set value to 1.
 * Wait for 2 and set value to 3, and close it by disabling
 */
TEST(CounterWaitHandlerTest, wait_value)
{
    // Wait for 1 and set value to 1.
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_value(1);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(1);

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for 2 and set value to 3, and close it by disabling
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_value(2);
                ASSERT_EQ(reason, AwakeReason::DISABLED);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(3);

        // Disable waiter as it will not awake by value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.disable();

        // Wait for the thread to finish
        waiting_thread.join();
    }
}

/**
 * Test wait for a specific value.
 *
 * Wait for 3 and set value to 5.
 * Wait for -1 and let it awake alone as initial is 0.
 * Wait for 17 and set value to 13, and close it by disabling
 */
TEST(CounterWaitHandlerTest, wait_upper)
{
    // Wait for 3 and set value to 5.
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_upper_bound_threshold(3);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(5);

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for -1 and let it awake alone as initial is 0.
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_upper_bound_threshold(-1);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for 2 and set value to 3, and close it by disabling
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_upper_bound_threshold(17); // Wait for disable
                ASSERT_EQ(reason, AwakeReason::DISABLED);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(13);

        // Disable waiter as it will not awake by value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.disable();

        // Wait for the thread to finish
        waiting_thread.join();
    }
}

/**
 * Test wait for a specific value.
 *
 * Wait for -3 and set value to -5.
 * Wait for 1 and let it awake alone as initial is 0.
 * Wait for -17 and set value to -13, and close it by disabling
 */
TEST(CounterWaitHandlerTest, wait_lower)
{
    // Wait for 3 and set value to 5.
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_lower_bound_threshold(-3);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(-5);

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for -1 and let it awake alone as initial is 0.
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_lower_bound_threshold(1);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for 2 and set value to 3, and close it by disabling
    {
        CounterWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_lower_bound_threshold(-17); // Wait for disable
                ASSERT_EQ(reason, AwakeReason::DISABLED);
            }
            );

        // Wait a bit for waiter to change value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.set_value(-13);

        // Disable waiter as it will not awake by value
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));
        waiter.disable();

        // Wait for the thread to finish
        waiting_thread.join();
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}