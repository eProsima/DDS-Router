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

#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/time/PausableTimer.hpp>


namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

constexpr const eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME = 10;
constexpr const eprosima::ddsrouter::utils::Duration_ms TIME_WAIT_TEST = 200;

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */


using namespace eprosima::ddsrouter::utils;

/**
 * Create a Timer and check its values
 *
 * CASES:
 * - create it playing
 * - create it paused
 */
TEST(PausableTimerTest, create)
{
    // create it playing
    {
        PausableTimer timer(true);
        ASSERT_TRUE(timer.playing());
        sleep_for(test::RESIDUAL_TIME);
        ASSERT_LT(0, timer.elapsed());
    }

    // create it paused
    {
        PausableTimer timer(false);
        ASSERT_FALSE(timer.playing());
        ASSERT_EQ(0, timer.elapsed());
    }
}

/**
 * Call repeated times to play and pause without failing
 *
 * CASES:
 * - play multiple times
 * - pause multiple times
 * - play and pause repeated
 */
TEST(PausableTimerTest, repeat_play_pause)
{
    // play multiple times
    {
        PausableTimer timer(true);
        ASSERT_TRUE(timer.playing());

        timer.play();
        ASSERT_TRUE(timer.playing());

        timer.play();
        timer.play();
        timer.play();
        ASSERT_TRUE(timer.playing());
    }

    // pause multiple times
    {
        PausableTimer timer(false);
        ASSERT_FALSE(timer.playing());

        timer.pause();
        ASSERT_FALSE(timer.playing());

        timer.pause();
        timer.pause();
        timer.pause();
        ASSERT_FALSE(timer.playing());
    }

    // play and pause repeated
    {
        PausableTimer timer(false);
        ASSERT_FALSE(timer.playing());

        timer.play();
        timer.play();
        ASSERT_TRUE(timer.playing());

        timer.pause();
        ASSERT_FALSE(timer.playing());

        timer.pause();
        timer.pause();
        ASSERT_FALSE(timer.playing());

        timer.play();
        ASSERT_TRUE(timer.playing());
    }
}

/**
 * Test play and pause work correctly
 *
 * STEPS:
 * - Start playing for N ms
 * - Pause for 2N ms
 * - Check time
 * - Play again for 2N ms
 * - Check time
 * - Reset while playing and wait N ms
 * - Check time
 * - Pause and reset while paused and wait N ms
 * - Check time
 */
TEST(PausableTimerTest, play_and_pause)
{
    PausableTimer timer(true);

    // Start playing for N ms
    sleep_for(test::TIME_WAIT_TEST);

    // Pause for 2N ms
    timer.pause();
    sleep_for(2 * test::TIME_WAIT_TEST);

    // Check time
    ASSERT_GE(timer.elapsed_ms(), test::TIME_WAIT_TEST);
    ASSERT_LT(timer.elapsed_ms(), 2 * test::TIME_WAIT_TEST);

    // Play again for 2N ms
    timer.play();
    sleep_for(2 * test::TIME_WAIT_TEST);

    // Check time
    auto time_elapsed = timer.elapsed_ms();
    ASSERT_GE(time_elapsed, 3 * test::TIME_WAIT_TEST);
    ASSERT_LT(time_elapsed, 4 * test::TIME_WAIT_TEST);

    // Reset while playing and wait N ms
    timer.reset();
    sleep_for(test::TIME_WAIT_TEST);

    // Check time
    ASSERT_GE(timer.elapsed_ms(), test::TIME_WAIT_TEST);
    ASSERT_LT(timer.elapsed_ms(), 2 * test::TIME_WAIT_TEST);

    // Pause and reset while paused and wait N ms
    timer.pause();
    timer.reset();
    sleep_for(test::TIME_WAIT_TEST);

    // Check time
    ASSERT_EQ(0, timer.elapsed_ms());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
