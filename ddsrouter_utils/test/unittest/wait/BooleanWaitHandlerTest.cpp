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

#include <ddsrouter_utils/wait/BooleanWaitHandler.hpp>

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
 * \c Create a BooleanWaitHandler and check its internal values
 *
 * CASES:
 * - open enable
 * - open disable
 * - close enable
 * - close disable
 */
TEST(BooleanWaitHandlerTest, create_wait_handler)
{
    // open enable
    {
        BooleanWaitHandler waiter(true, true);

        ASSERT_TRUE(waiter.is_open());
        ASSERT_TRUE(waiter.enabled());
    }

    // open disable
    {
        BooleanWaitHandler waiter(true, false);

        ASSERT_TRUE(waiter.is_open());
        ASSERT_FALSE(waiter.enabled());
    }

    // close enable
    {
        BooleanWaitHandler waiter(false, true);

        ASSERT_FALSE(waiter.is_open());
        ASSERT_TRUE(waiter.enabled());
    }

    // close disable
    {
        BooleanWaitHandler waiter(false, false);

        ASSERT_FALSE(waiter.is_open());
        ASSERT_FALSE(waiter.enabled());
    }
}

/**
 * Check to open and close the object several times and see the internal value is correct.
 *
 * STEPS:
 * - open
 * - close
 * - open twice
 * - close once
 */
TEST(BooleanWaitHandlerTest, open_close)
{
    BooleanWaitHandler waiter(false);

    // open
    waiter.open();
    ASSERT_TRUE(waiter.is_open());

    // close
    waiter.close();
    ASSERT_FALSE(waiter.is_open());

    // open twice
    waiter.open();
    waiter.open();
    ASSERT_TRUE(waiter.is_open());

    // close once
    waiter.close();
    ASSERT_FALSE(waiter.is_open());
}

/**
 * Create a thread that waits in a waiter and finished by opening waiter
 *
 * Wait is done in main thread, just to avoid errors and check test correctly.
 */
TEST(BooleanWaitHandlerTest, wait_for_open)
{
    BooleanWaitHandler waiter(false); // It starts closed and enabled

    // Create a thread that waits for the waiter to open
    std::thread waiting_thread(
        [&waiter]()
        {
            // Awake new thread by opening waiter
            // Note that if this opening is done before the wait,
            // it will jut not wait but awake reason will be correct.
            waiter.open();
        }
        );

    AwakeReason reason = waiter.wait(); // Wait forever
    ASSERT_EQ(reason, AwakeReason::condition_met);

    // Wait for the thread to finish
    waiting_thread.join();
}

/**
 * Wait for a small amount of time and check the wait returns timeout
 */
TEST(BooleanWaitHandlerTest, wait_for_timeout)
{
    BooleanWaitHandler waiter(false); // It starts closed and enabled

    // Wait for a small amount of time
    AwakeReason reason = waiter.wait(test::RESIDUAL_TIME_TEST);

    // Check it has been awaken from timeout
    ASSERT_EQ(reason, AwakeReason::timeout);
}

/**
 * Create a thread that disables the waiter while main one waits.
 */
TEST(BooleanWaitHandlerTest, wait_for_disable)
{
    BooleanWaitHandler waiter(false); // It starts closed and enabled

    // Create a thread that waits for the waiter to open
    std::thread waiting_thread(
        [&waiter]()
        {
            // Awake new thread by disabling waiter
            waiter.disable();
        }
        );

    AwakeReason reason = waiter.wait(); // Wait forever
    ASSERT_EQ(reason, AwakeReason::disabled);

    // Wait for the thread to finish
    waiting_thread.join();
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
