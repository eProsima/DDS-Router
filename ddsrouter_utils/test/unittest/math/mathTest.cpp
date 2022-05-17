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

void compare_fast_module(
        unsigned int dividend,
        unsigned int divisor)
{
    ASSERT_EQ(fast_module(dividend, divisor), dividend % divisor) << dividend << " % " << divisor;
}

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test \c is_file_accesible method
 *
 * CASES:
 * - dividend lower than divisor
 * - dividend equal to divisor
 * - divisor = 2
 * - divisor = 2^N
 * - divisor even no 2^N
 * - divisor odd
 */
TEST(mathTest, fast_module)
{
    // dividend lower than divisor
    {
        test::compare_fast_module(3, 4);
        test::compare_fast_module(0, 4);
        test::compare_fast_module(101223, 20921341);
    }

    // dividend equal to divisor
    {
        test::compare_fast_module(3, 3);
        test::compare_fast_module(4, 4);
        test::compare_fast_module(66666, 66666);
    }

    // divisor = 2
    {
        test::compare_fast_module(3, 2);
        test::compare_fast_module(4, 2);
        test::compare_fast_module(66666, 2);
        test::compare_fast_module(431253426, 2);
    }

    // divisor = 2^N
    {
        test::compare_fast_module(3, 4);
        test::compare_fast_module(32, 8);
        test::compare_fast_module(66666, 128);
        test::compare_fast_module(431253426, 2048);
    }

    // divisor even no 2^N
    {
        test::compare_fast_module(3, 8);
        test::compare_fast_module(12, 10);
        test::compare_fast_module(66666, 120);
        test::compare_fast_module(431253426, 2040);
    }

    // divisor odd
    {
        test::compare_fast_module(3, 5);
        test::compare_fast_module(12, 11);
        test::compare_fast_module(66666, 127);
        test::compare_fast_module(431253426, 2041);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
