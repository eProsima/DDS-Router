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

#include <ddsrouter_utils/types/Fuzzy.hpp>

namespace test {

// Dummy class to test Fuzzy class
struct A
{
    A() : x(-1) {}
    A(int x) : x(x) {}
    bool operator==(const A& other) const { return x == other.x; }
    int x;
};

} /* namespace test */

using namespace eprosima::ddsrouter::utils;

/**
 * Test Fuzzy default_construct
 *
 * CASES:
 * - int
 * - string
 * - A
 */
TEST(fuzzyTest, default_construct)
{
    // int
    {
        Fuzzy<int> f;
        // ASSERT_EQ(f.get_reference(), 0);  could be any
        ASSERT_FALSE(f.is_set());
    }

    // string
    {
        Fuzzy<std::string> f;
        ASSERT_EQ(f.get_reference(), "");
        ASSERT_FALSE(f.is_set());
    }

    // A
    {
        Fuzzy<test::A> f;
        ASSERT_EQ(f.get_reference().x, -1);
        ASSERT_FALSE(f.is_set());
    }
}

/**
 * Test Fuzzy construct
 *
 * CASES:
 * - int
 * - string
 * - A
 */
TEST(fuzzyTest, construct)
{
    // int
    {
        Fuzzy<int> f(3);
        ASSERT_EQ(f.get_reference(), 3);
        ASSERT_TRUE(f.is_set());
    }

    // string
    {
        Fuzzy<std::string> f("hello");
        ASSERT_EQ(f.get_reference(), "hello");
        ASSERT_TRUE(f.is_set());
    }

    // A
    {
        Fuzzy<test::A> f(2);
        ASSERT_EQ(f.get_reference().x, 2);
        ASSERT_TRUE(f.is_set());
    }
}

/**
 * Test Fuzzy assign operator
 *
 * CASES:
 * - int
 * - string
 * - A
 */
TEST(fuzzyTest, assign_operator)
{
    // int
    {
        Fuzzy<int> f;
        f = 3;
        ASSERT_EQ(f.get_reference(), 3);
        ASSERT_TRUE(f.is_set());
    }

    // string
    {
        Fuzzy<std::string> f;
        f = std::string("hello");
        ASSERT_EQ(f.get_reference(), "hello");
        ASSERT_TRUE(f.is_set());
    }

    // A
    {
        Fuzzy<test::A> f;
        f = test::A(2);
        ASSERT_EQ(f.get_reference().x, 2);
        ASSERT_TRUE(f.is_set());
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
