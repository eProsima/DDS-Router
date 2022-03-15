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

#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/Time.hpp>

using namespace eprosima::ddsrouter::utils;

/**
 * For each case, creates a timer, waits for that amount of milliseconds and check
 * that the result of \c elapsed is bigger than the wait time.
 *
 * CASES:
 * - 5 ms
 * - 50 ms
 * - 500 ms
 */
TEST(TimerTest, elapsed)
{
    // Cases
    std::vector<double> cases({5, 50, 500});

    for (double wait_time : cases)
    {
        Timer timer;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(wait_time)));
        ASSERT_GE(timer.elapsed(), wait_time);
    }
}

/**
 * For each case, creates a timer, waits for that amount of milliseconds and check
 * that the result of \c elapsed_ms is bigger than the wait time.
 *
 * CASES:
 * - 5 ms
 * - 50 ms
 * - 500 ms
 */
TEST(TimerTest, elapsed_ms)
{
    // Cases
    std::vector<Duration_ms> cases({5, 50, 500});

    for (Duration_ms wait_time : cases)
    {
        Timer timer;
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        ASSERT_GE(timer.elapsed_ms(), wait_time);
    }
}

/**
 * For each case, creates a timer, waits for that amount of milliseconds, reset timer
 * and check that elapsed time is less than the time elapsed since the creation.
 *
 * @warning for very short cases it could happen that calling reset and elapsed takes
 * more time than the case, so test fails
 *
 * CASES:
 * - 5 ms
 * - 50 ms
 * - 500 ms
 */
TEST(TimerTest, reset)
{
    // Cases
    std::vector<double> cases({5, 50, 500});

    for (double wait_time : cases)
    {
        Timer timer;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(wait_time)));

        // Reset timer
        timer.reset();
        ASSERT_LT(timer.elapsed(), wait_time);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
