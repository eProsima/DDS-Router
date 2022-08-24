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

// namespace test {

// ENUMERATION_BUILDER(
//     TestCustomEnum,
//     el0,
//     el1,
//     other_element,
//     noElement_atAll_00
// );

// constexpr unsigned int NUMBER_OF_ELEMENTS = 4;

// const std::array<TestCustomEnum::Enum, NUMBER_OF_ELEMENTS> enum_values =
// {
//     TestCustomEnum::Enum::el0,
//     TestCustomEnum::Enum::el1,
//     TestCustomEnum::Enum::other_element,
//     TestCustomEnum::Enum::noElement_atAll_00,
// };

// const std::array<TestCustomEnum, NUMBER_OF_ELEMENTS> class_values =
// {
//     TestCustomEnum(TestCustomEnum::Enum::el0),
//     TestCustomEnum(TestCustomEnum::Enum::el1),
//     TestCustomEnum(TestCustomEnum::Enum::other_element),
//     TestCustomEnum(TestCustomEnum::Enum::noElement_atAll_00),
// };

// const std::array<std::string, NUMBER_OF_ELEMENTS> string_values =
// {
//     "el0",
//     "el1",
//     "other_element",
//     "noElement_atAll_00",
// };

// } /* namespace test */

// /**
//  * Construct enumeration of type TestCustomEnum from its values.
//  *
//  * CASES:
//  * x each element inside
//  */
// TEST(enumerationBuilderMacrosTest, construct_enum)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         test::TestCustomEnum value(test::enum_values[i]);
//         ASSERT_TRUE(value == test::class_values[i]);
//         ASSERT_TRUE(value == test::enum_values[i]);
//     }
// }

// /**
//  * Construct enumeration of type TestCustomEnum from a string.
//  *
//  * CASES:
//  * x each element inside
//  * - string not in enumeration
//  */
// TEST(enumerationBuilderMacrosTest, construct_string)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         test::TestCustomEnum value(test::string_values[i]);
//         ASSERT_TRUE(value == test::class_values[i]);
//         ASSERT_TRUE(value == test::enum_values[i]);
//     }

//     // string not in enumeration
//     ASSERT_THROW(test::TestCustomEnum("noElement_atAll_0") , eprosima::ddsrouter::utils::InitializationException);
// }

// /**
//  * Compare to_string with expected result
//  *
//  * CASES:
//  * x each element inside
//  */
// TEST(enumerationBuilderMacrosTest, to_string)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         ASSERT_EQ(test::class_values[i].to_string(), test::string_values[i]);
//     }
// }

// /**
//  * Compare to string cast operator with expected result
//  *
//  * CASES:
//  * x each element inside
//  */
// TEST(enumerationBuilderMacrosTest, string_operator)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         ASSERT_EQ(static_cast<std::string>(test::class_values[i]), test::string_values[i]);
//     }
// }

// /**
//  * Compare to int cast operator with expected result
//  *
//  * CASES:
//  * x each element inside
//  */
// TEST(enumerationBuilderMacrosTest, int_operator)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         ASSERT_EQ(static_cast<int>(test::class_values[i]), i);
//     }
// }

// /**
//  * Compare to int cast operator with expected result
//  *
//  * CASES:
//  * x each element inside
//  */
// TEST(enumerationBuilderMacrosTest, serializator)
// {
//     // x each element inside
//     for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
//     {
//         std::stringstream ss;
//         ss << test::class_values[i];
//         ASSERT_EQ(ss.str(), test::string_values[i]);
//     }
// }

namespace test {

ENUMERATION_BUILDER(
    TestCustomEnum,
    el0,
    el1,
    other_element,
    noElement_atAll_00
);

ENUMERATION_BUILDER(
    TestCustomOtherEnum,
    el_0,
);

constexpr unsigned int NUMBER_OF_ELEMENTS = 4;

const std::array<TestCustomEnum, NUMBER_OF_ELEMENTS> enum_values =
{
    TestCustomEnum::el0,
    TestCustomEnum::el1,
    TestCustomEnum::other_element,
    TestCustomEnum::noElement_atAll_00,
};

const std::array<std::string, NUMBER_OF_ELEMENTS> string_values =
{
    "el0",
    "el1",
    "other_element",
    "noElement_atAll_00",
};

} /* namespace test */

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
    for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
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
    for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
    {
        ASSERT_EQ(test::to_string(test::enum_values[i]), test::string_values[i]);
    }
}

/**
 * Compare to int cast operator with expected result
 *
 * CASES:
 * x each element inside
 */
TEST(enumerationBuilderMacrosTest, serializator)
{
    // x each element inside
    for (int i = 0; i < test::NUMBER_OF_ELEMENTS; i++)
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
