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

#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>

#include <ddsrouter_utils/event/PeriodicEventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace test {

eprosima::ddsrouter::utils::Duration_ms DEFAULT_TIME_TEST = 1000u;
eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME_TEST = 10u;
eprosima::ddsrouter::utils::Duration_ms LONG_TIME_TEST = 5000u;

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::event;

/**
 * @brief Create an object of PeriodicEventHandler and let it be destroyed.
 *
 * This test may crush if fails.
 *
 * CASES:
 * - with callback
 * - without callback
 *
 */
TEST(PeriodicEventHandlerTest, trivial_create_handler)
{
    // with callback
    {
        PeriodicEventHandler handler(
            []()
            {
                /* empty callback */ },
            test::DEFAULT_TIME_TEST);
    }

    // without callback
    {
        PeriodicEventHandler handler(test::DEFAULT_TIME_TEST);
    }
}

/**
 * @brief Create a handler object and check that waits at least the time given before let thread continue.
 *
 * CASES:
 * - 5ms
 * - 100ms
 * - 1s
 */
TEST(PeriodicEventHandlerTest, wait_handler)
{
    std::vector<utils::Duration_ms> times = {5u, 100u, 1000u};

    for (utils::Duration_ms time : times)
    {
        // Get started time
        utils::Timer timer;

        // Create handler and wait
        PeriodicEventHandler handler(
            []()
            {
                /* empty callback */ },
            time);

        // Wait for the period time
        handler.wait_for_event();

        ASSERT_GE(timer.elapsed_ms(), time) << timer.elapsed_ms() << "<" << time;
    }
}

/**
 * @brief Check handler callback is called coherently.
 *
 * Create a Handler.
 * Wait for twice the time of the timer.
 * Check that there has been at least two calls.
 *
 * CASES:
 * - 5ms
 * - 50ms
 * - 500ms
 */
TEST(PeriodicEventHandlerTest, handler_period)
{
    std::vector<utils::Duration_ms> times = {5u, 50u, 500u};

    for (utils::Duration_ms time : times)
    {
        std::atomic<uint32_t> calls(0);

        // Create handler and wait
        PeriodicEventHandler handler(
            [&calls]()
            {
                calls++;
            },
            time);

        // Wait twice for the period time
        handler.wait_for_event(2);

        ASSERT_GE(calls.load(), 2) << calls;
    }
}

/**
 * @brief Check that creating a handler without callback does not start counter
 *
 */
TEST(PeriodicEventHandlerTest, handler_period_without_callback)
{
    utils::Duration_ms handler_period = 10u;
    PeriodicEventHandler handler(handler_period);

    std::this_thread::sleep_for(std::chrono::milliseconds(handler_period * 5)); // It should have activated 5 times

    // Check it has not been activated
    ASSERT_EQ(handler.event_count(), 0);
}

/**
 * @brief Check handler limit cases
 *
 * CASES:
 * - 1ms period
 * - 1ms period and instant close
 *
 */
TEST(PeriodicEventHandlerTest, limit_cases)
{
    // 1ms period
    {
        PeriodicEventHandler handler([]()
                {
                    /* empty callback */ }, utils::Duration_ms(1u));

        // Wait 100 events for the period time
        handler.wait_for_event(100);

        ASSERT_GE(handler.event_count(), 100);
    }

    // 1ms period and instant close
    {
        PeriodicEventHandler handler([]()
                {
                    /* empty callback */ }, utils::Duration_ms(1u));
    }
}

/**
 * @brief Check negative construct cases for Handler
 *
 * CASES:
 * - time 0
 */
TEST(PeriodicEventHandlerTest, negative_cases)
{
    // time 0
    {
        // Without callback
        ASSERT_THROW(PeriodicEventHandler(0), utils::InitializationException);
        // With callback
        ASSERT_THROW(PeriodicEventHandler([]()
                {
                    /* empty callback */ }, 0), utils::InitializationException);
    }
}

/**
 * @brief Check tat destroying a PeriodicEventHandler do not wait till period to be destroyed
 *
 * Create a PeriodicEventHandler with T time and destroy it afterwards.
 * Check that time elapsed since the creation and the destruction is less than T time.
 */
TEST(PeriodicEventHandlerTest, not_wait_in_destruction)
{
    utils::Duration_ms handler_period = test::DEFAULT_TIME_TEST;

    // Create timer to measure time
    utils::Timer timer;
    {

        // Create object and let it start by waiting for first event
        PeriodicEventHandler handler( []()
                {
                    /* empty callback */ }, handler_period);
        handler.wait_for_event();

        // Destroy handler
    }

    utils::Duration_ms time_elapsed = timer.elapsed_ms();

    // Check that the time elapsed is smaller than the time required to wait for first event AND to stop handler
    ASSERT_LT(time_elapsed, handler_period * 2) << time_elapsed;
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
