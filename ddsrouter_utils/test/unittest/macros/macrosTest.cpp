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

#include <ddsrouter_utils/macros/macros.hpp>

/**
 * Test \c STRINGIFY macro
 *
 * Cases:
 *  random string
 *  variable name
 */
TEST(macrosTest, stringify)
{
    // random string
    {
        ASSERT_EQ(
            std::string(STRINGIFY(abcdefghijkl)),
            std::string("abcdefghijkl")
        );

        ASSERT_EQ(
            std::string(STRINGIFY(k1)),
            std::string("k1")
        );
    }

    // variable name
    {
        // Integer
        int _random_value = 0;
        ASSERT_EQ(
            std::string(STRINGIFY(_random_value)),
            std::string("_random_value")
        );

        // String
        std::string _random_string = "other value";
        ASSERT_EQ(
            std::string(STRINGIFY(_random_string)),
            std::string("_random_string")
        );
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
