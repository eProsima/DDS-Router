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

#include <sstream>

#include <ddsrouter_utils/macros/custom_enumeration.hpp>

namespace test {

// This will create a TestCustomEnum enum class with these values.
ENUMERATION_BUILDER(
    TestCustomEnum,
    el0,
    el1,
    other_element,
    noElement_atAll_00
);

// Number of elements in TestCustomEnum
constexpr unsigned int NUMBER_OF_ELEMENTS = 4;

// Each of the elements of TestCustomEnum
const std::array<TestCustomEnum, NUMBER_OF_ELEMENTS> enum_values =
{
    TestCustomEnum::el0,
    TestCustomEnum::el1,
    TestCustomEnum::other_element,
    TestCustomEnum::noElement_atAll_00,
};

// Each of the names of TestCustomEnum
const std::array<std::string, NUMBER_OF_ELEMENTS> string_values =
{
    "el0",
    "el1",
    "other_element",
    "noElement_atAll_00",
};

// This tests that two calls to ENUMERATION_BUILDER are possible.
ENUMERATION_BUILDER(
    TestCustomOtherEnum,
    el0
);

// This should be forbidden and would not compile
/*
ENUMERATION_BUILDER(
    TestEmptyEnum
);
*/

} /* namespace test */

/**
 * This test only shows how to use the ENUMERATION_BUILDER macro.
 */
TEST(enumerationBuilderMacrosTest, show_how)
{
    // Get a new element of value el0
    test::TestCustomEnum value = test::TestCustomEnum::el0;

    // Compare string of value
    ASSERT_EQ("el0", test::to_string(value));

    // Get it from a string
    value = test::from_string_TestCustomEnum("el1");

    // Compare string of value
    ASSERT_EQ("el1", test::to_string(value));
}

/**
 * Check the N_VALUES variable
 */
TEST(enumerationBuilderMacrosTest, n_values)
{
    ASSERT_EQ(test::N_VALUES_TestCustomEnum, test::NUMBER_OF_ELEMENTS);
}

/**
 * Construct enumeration of type TestCustomEnum from a string.
 *
 * CASES:
 * x each element inside
 * - string not in enumeration
 */
TEST(enumerationBuilderMacrosTest, from_string)
{
    // x each element inside
    for (int i = 0; i < test::N_VALUES_TestCustomEnum; i++)
    {
        test::TestCustomEnum value = test::from_string_TestCustomEnum(test::string_values[i]);
        ASSERT_TRUE(value == test::enum_values[i]);
    }

    // string not in enumeration
    ASSERT_THROW(test::from_string_TestCustomEnum("el_0") , eprosima::ddsrouter::utils::InitializationException);
}

/**
 * Compare to_string with expected result
 *
 * CASES:
 * x each element inside
 */
TEST(enumerationBuilderMacrosTest, to_string)
{
    // x each element inside
    for (int i = 0; i < test::N_VALUES_TestCustomEnum; i++)
    {
        ASSERT_EQ(test::to_string(test::enum_values[i]), test::string_values[i]);
    }
}

/**
 * Compare serializator method of each value of enumeration
 *
 * CASES:
 * x each element inside
 */
TEST(enumerationBuilderMacrosTest, serializator)
{
    // x each element inside
    for (int i = 0; i < test::N_VALUES_TestCustomEnum; i++)
    {
        std::stringstream ss;
        ss << test::enum_values[i];
        ASSERT_EQ(ss.str(), test::string_values[i]);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
