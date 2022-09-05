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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/macros/recursive_macros.hpp>
#include <ddsrouter_utils/utils.hpp>

/**
 * Test \c COUNT_ARGUMENTS macro
 *
 * Cases:
 *  0
 *  1
 *  5
 *  9
 */
TEST(recursiveMacrosTest, count_arguments)
{
    // 0
    {
        ASSERT_EQ(
            0,
            COUNT_ARGUMENTS()
        );
    }

    // 1
    {
        ASSERT_EQ(
            1,
            COUNT_ARGUMENTS(
                "el1"
            )
        );
    }

    // 5
    {
        ASSERT_EQ(
            5,
            COUNT_ARGUMENTS(
                "el1", "el2", "el3", "el4", "el5"
            )
        );
    }

    // 9
    {
        ASSERT_EQ(
            9,
            COUNT_ARGUMENTS(
                "el1", "el2", "el3", "el4", "el5",
                "el1", "el2", "el3", "el4"
            )
        );
    }
}

/**
 * Test \c APPLY_MACRO_FOR_EACH macro
 *
 * Cases:
 *  addition
 *  string concatenation
 */
TEST(recursiveMacrosTest, apply_macro_for_each)
{

#define ADD_1(x) x += 1;

    // addition
    {
        int x = 1;
        int y = 2;
        int z = 3;

        APPLY_MACRO_FOR_EACH(
            ADD_1,
            x,
            y,
            z
        );

        ASSERT_EQ(x, 2);
        ASSERT_EQ(y, 3);
        ASSERT_EQ(z, 4);
    }

#define TO_LOWERCASE(x) eprosima::ddsrouter::utils::to_lowercase(x);

    // string concatenation
    {
        std::string hello = "HELLO";
        std::string bye = "ByE";

        APPLY_MACRO_FOR_EACH(
            TO_LOWERCASE,
            hello,
            bye
        );

        ASSERT_EQ(hello, "hello");
        ASSERT_EQ(bye, "bye");
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
