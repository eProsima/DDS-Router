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

#include <algorithm>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <climits>

#include <ddsrouter_utils/memory/owner_ptr.hpp>

using namespace eprosima::ddsrouter::utils;

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace test {

struct TestClass
{
    TestClass()
        : int_value(42)
        , string_value("Douglas Adams")
        , vector_value({'T', 'U', 'H', 'G'})
    {
    }

    int get_int()
    {
        return int_value;
    }

    std::string get_str()
    {
        return string_value;
    }

    std::vector<char> get_vec()
    {
        return vector_value;
    }

    int int_value;
    std::string string_value;
    std::vector<char> vector_value;
};

} /* namespace test */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Create a default owner ptr
 */
TEST(OwnerPtrTest, owner_ptr_default_constructor)
{
    OwnerPtr<int> ptr;
    ASSERT_FALSE(ptr);
}

/**
 * Create a OwnerPtr with type int and access it
 *
 * CASES:
 * - Multiple numbers to check
 */
TEST(OwnerPtrTest, owner_ptr_access_int)
{
    std::vector<int> test_cases = {0, 1, 10, -5, INT_MAX};

    for (int test_case : test_cases)
    {
        OwnerPtr<int> ptr(new int(test_case));

        EXPECT_EQ(test_case, *ptr);
    }
}

/**
 * Create a OwnerPtr with type std::string and access it
 *
 * CASES:
 * - Multiple numbers to check
 */
TEST(OwnerPtrTest, owner_ptr_access_string)
{
    std::vector<std::string> test_cases = {
        "HelloPtr",
        "",
        "ThisIsAComplex,Long\nHardTestStringToCheckTheOwnerPtrWorks.",
    };

    for (std::string test_case : test_cases)
    {
        OwnerPtr<std::string> ptr(
            new std::string(test_case));

        EXPECT_EQ(test_case, *ptr);
    }
}

/**
 * Create a \c TestClass object (always initialized with same values)
 *
 * STEPS:
 * - Create ptr from an object of TestClass
 * - Create an in-place object of TestClass (has same parameters)
 * - Access each parameter of both and check they are the same
 * - Modify a parameter of ptr and check it has changed
 */
TEST(OwnerPtrTest, owner_ptr_access_class)
{
    OwnerPtr<test::TestClass> ptr(
        new test::TestClass());

    test::TestClass tester_object; // Every TestClass object is initialized with same values

    EXPECT_EQ(tester_object.get_int(), ptr->get_int());
    EXPECT_EQ(tester_object.get_str(), ptr->get_str());
    EXPECT_EQ(tester_object.get_vec(), ptr->get_vec());

    // Modify int
    ptr->int_value += 1;
    EXPECT_NE(tester_object.get_int(), ptr->get_int());
}

/**
 * Create a OwnerPtr with specific deleter and check it has been called, and not the delete function.
 * This deleter will increase a variable to check it has been called.
 */
TEST(OwnerPtrTest, owner_ptr_custom_deleter)
{
    // Variable to store deleter calls
    int deleter_calls = 0;

    // Create Ptr object from int with custom deleter
    OwnerPtr<int> ptr(
        new int(42),
        [&deleter_calls](int*)
        {
            deleter_calls++;
        });

    // Get access to int reference
    int& actual_value_inside = *ptr;

    // Remove ptr
    ptr.reset();

    // Check the deleter has been called
    ASSERT_EQ(deleter_calls, 1);

    // As deleter did not destroy object, it should still be accessible
    ASSERT_EQ(actual_value_inside, 42);
}

/**
 * Check reset method
 *
 * STEPS:
 * - Create a void ptr
 * - Reset with a new value
 * - Reset with nothing
 * - Reset with specific deleter
 * - Reset with nothing
 */
TEST(OwnerPtrTest, owner_ptr_reset)
{
    // Variable to store deleter calls
    int deleter_calls = 0;

    // Create a void ptr
    OwnerPtr<int> ptr;

    // Reset with a new value
    ptr.reset(new int(1));
    ASSERT_EQ(*ptr, 1);

    // Reset with nothing
    ptr.reset();
    ASSERT_FALSE(ptr);

    // Reset with specific deleter
    ptr.reset(
        new int(2),
        [&deleter_calls](int* x)
        {
            deleter_calls++;
            delete x;
        });
    ASSERT_EQ(*ptr, 2);

    // Reset with nothing
    ptr.reset();
    ASSERT_FALSE(ptr);
    ASSERT_EQ(deleter_calls, 1);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
