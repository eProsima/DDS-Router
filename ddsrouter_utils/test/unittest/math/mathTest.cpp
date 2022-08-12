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

#include <algorithm>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/math/math.hpp>

using namespace eprosima::ddsrouter::utils;

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

uint32_t NUMBERS_TO_TEST = 1000;
uint32_t NUMBERS_TO_TEST_SHORT = 100;

bool compare_is_even(
        uint32_t number)
{
    return (
        is_even(number)
        ==
        number % 2 == 0);
}

bool compare_fast_module(
        uint32_t dividend,
        uint32_t divisor)
{
    return (
        fast_module(dividend, divisor)
        ==
        dividend % divisor);
}

bool compare_fast_division(
        uint32_t dividend,
        uint32_t divisor)
{
    return (
        fast_division(dividend, divisor)
        ==
        dividend / divisor);
}

bool compare_arithmetic_progression_sum(
        uint32_t lowest,
        uint32_t interval,
        uint32_t steps)
{
    uint32_t current_number = lowest;
    uint32_t real_result = 0;
    for (int i = 0; i < steps; ++i)
    {
        real_result += current_number;
        current_number += interval;
    }

    return (
        arithmetic_progression_sum(lowest, interval, steps)
        ==
        real_result);
}

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test \c is_even method
 */
TEST(mathTest, is_even)
{
    // calculate module in many cases
    for (uint32_t number = 0; number < test::NUMBERS_TO_TEST; ++number)
    {
        ASSERT_TRUE(test::compare_is_even(number))
            << number;
    }
}

/**
 * Test \c fast_module method
 */
TEST(mathTest, fast_module)
{
    // calculate module in many cases
    for (uint32_t dividend = 0; dividend < test::NUMBERS_TO_TEST; ++dividend)
    {
        for (uint32_t divisor = 1; divisor < test::NUMBERS_TO_TEST; ++divisor)
        {
            ASSERT_TRUE(test::compare_fast_module(dividend, divisor))
                << dividend << " % " << divisor;
        }
    }
}

/**
 * Test \c fast_division method
 */
TEST(mathTest, fast_division)
{
    // calculate module in many cases
    for (uint32_t dividend = 0; dividend < test::NUMBERS_TO_TEST; ++dividend)
    {
        for (uint32_t divisor = 1; divisor < test::NUMBERS_TO_TEST; ++divisor)
        {
            ASSERT_TRUE(test::compare_fast_division(dividend, divisor))
                << dividend << " % " << divisor;
        }
    }
}

/**
 * Test \c arithmetic_progression_sum method
 */
TEST(mathTest, arithmetic_progression_sum)
{
    // calculate module in many cases
    for (uint32_t lowest = 0; lowest < test::NUMBERS_TO_TEST_SHORT; ++lowest)
    {
        for (uint32_t interval = 1; interval < test::NUMBERS_TO_TEST_SHORT; ++interval)
        {
            for (uint32_t steps = 1; steps < test::NUMBERS_TO_TEST_SHORT; ++steps)
            {
                ASSERT_TRUE(test::compare_arithmetic_progression_sum(lowest, interval, steps))
                    << lowest << " , " << interval << " , " << steps;
            }
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
