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

#include <ddsrouter_event/wait/IntWaitHandler.hpp>

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
 * Create a \c IntWaitHandler and check its internal values with different numbers
 *
 * Change the internal values and check if the values are changed
 */
TEST(CounterWaitHandlerTest, value_handler)
{
    std::vector<IntWaitHandlerType> counter_values = {0, 1, 12345, -1, -99999};

    for (IntWaitHandlerType i : counter_values)
    {
        IntWaitHandler waiter(i);

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
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_equal(1);
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
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_equal(2);
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
 * Wait for ge 13 and set value to 13
 * Wait for 17 and set value to 13, and close it by disabling
 */
TEST(CounterWaitHandlerTest, wait_upper)
{
    // Wait for 3 and set value to 5.
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_greater_than(3);
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
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_greater_than(-1);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for ge 13 and set value to 13
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_greater_equal_than(13);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );
        waiter.set_value(13);

        // Wait for the thread to finish
        waiting_thread.join();
    }

    //  Wait for 17 and set value to 13, and close it by disabling
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_greater_than(17); // Wait for disable
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
 * Wait for le 13 and set value to 13
 * Wait for -17 and set value to -13, and close it by disabling
 */
TEST(CounterWaitHandlerTest, wait_lower)
{
    // Wait for 3 and set value to 5.
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                // Note that if this wait occurs after set_value
                // it will jut not wait but awake reason will be correct.
                AwakeReason reason = waiter.wait_lower_than(-3);
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
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_lower_than(1);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for le 13 and set value to 13
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_lower_equal_than(13);
                ASSERT_EQ(reason, AwakeReason::CONDITION_MET);
            }
            );
        waiter.set_value(13);

        // Wait for the thread to finish
        waiting_thread.join();
    }

    // Wait for 2 and set value to 3, and close it by disabling
    {
        IntWaitHandler waiter(0); // It starts closed and enabled

        // Create a thread that waits for the waiter to open
        std::thread waiting_thread(
            [&waiter]()
            {
                // Wait in new thread
                AwakeReason reason = waiter.wait_lower_than(-17); // Wait for disable
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
