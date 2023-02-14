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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <cpp_utils/exception/ConfigurationException.hpp>

#include <ddspipe_yaml/YamlReader.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe::yaml;

/**
 * Check to get a boolean from a yaml
 *
 * CASES:
 *  true
 *  false
 *  fail with int
 */
TEST(YamlReaderScalarTest, get_scalar_bool)
{
    // true
    {
        Yaml yml(true);
        ASSERT_TRUE(YamlReader::get_scalar<bool>(yml));
    }

    // false
    {
        Yaml yml(false);
        ASSERT_FALSE(YamlReader::get_scalar<bool>(yml));
    }

    // fail with int
    {
        Yaml yml(0);
        ASSERT_THROW(YamlReader::get_scalar<bool>(yml), eprosima::utils::ConfigurationException);
    }
}

/**
 * Check to get an int from a yaml
 *
 * For each case are tested arbitrary numbers (power of 2) and negative ones in signed cases
 *
 * CASES:
 *  int
 *  uint
 *  uint16
 *  int64
 *  fail with bool
 */
TEST(YamlReaderScalarTest, get_scalar_int)
{
    // int
    {
        for (int i = 1; i < 0x1000; i *= 2)
        {
            ASSERT_EQ(YamlReader::get_scalar<int>(Yaml(i)), i) << i;
            ASSERT_EQ(YamlReader::get_scalar<int>(Yaml(-i)), -i) << -i;
        }
    }

    // uint
    {
        for (unsigned int i = 1; i < 0x1000; i *= 2)
        {
            ASSERT_EQ(YamlReader::get_scalar<unsigned int>(Yaml(i)), i) << i;
        }
    }

    // uint16
    {
        for (uint16_t i = 1; i < 0x1000; i *= 2)
        {
            ASSERT_EQ(YamlReader::get_scalar<uint16_t>(Yaml(i)), i) << i;
        }
    }

    // int64
    {
        for (int64_t i = 1; i < 0x1000; i *= 2)
        {
            ASSERT_EQ(YamlReader::get_scalar<int64_t>(Yaml(i)), i) << i;
            ASSERT_EQ(YamlReader::get_scalar<int64_t>(Yaml(-i)), -i) << -i;
        }
    }

    // fail with bool
    {
        ASSERT_THROW(YamlReader::get_scalar<int>(Yaml(false)), eprosima::utils::ConfigurationException);
    }
}

/**
 * Check to get a string from a yaml

 * CASES:
 *  empty string
 *  short string
 *  long string
 *  numeric string
 *  numeric
 *  bool
 */
TEST(YamlReaderScalarTest, get_scalar_string)
{
    // empty string
    {
        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml("")), "");
    }

    // short string
    {
        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml("short_text")), "short_text");
    }

    // long string
    {
        std::string st =
                "This text is pretty long, but will be "
                "concatenated into just a single string. "
                "The disadvantage is that you have to quote "
                "each part, and newlines must be literal as "
                "usual.";

        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml(st)), st);
    }

    // numeric string
    {
        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml("1234567890")), "1234567890");
    }

    // numeric
    {
        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml(111)), "111");
    }

    // bool
    {
        ASSERT_EQ(YamlReader::get_scalar<std::string>(Yaml(false)), "false");
    }
}

/**
 * Check to get a scalar type under a specific tag

 * CASES:
 *  bool
 *  int
 *  string
 *  no tag
 */
TEST(YamlReaderScalarTest, get_scalar_from_tag)
{
    const char* tag = "specific_tag";

    // bool
    {
        Yaml yml;
        yml[tag] = true;
        ASSERT_TRUE(YamlReader::get_scalar<bool>(yml, tag));
    }

    // int
    {
        Yaml yml;
        yml[tag] = 12345;
        ASSERT_EQ(YamlReader::get_scalar<int>(yml, tag), 12345);
    }

    // string
    {
        Yaml yml;
        yml[tag] = "this is a text";
        ASSERT_EQ(YamlReader::get_scalar<std::string>(yml, tag), "this is a text");
    }

    // no tag
    {
        Yaml yml;
        yml["other_tag"] = 3;
        ASSERT_THROW(YamlReader::get_scalar<int>(yml, tag), eprosima::utils::ConfigurationException);
    }
}

/**
 * Check that get_scalar fails in case that yaml is not a scalar
 *
 * CASES:
 *  null
 *  sequence
 *  map
 */
TEST(YamlReaderScalarTest, get_scalar_negative_cases)
{
    // null
    {
        Yaml yml;
        ASSERT_TRUE(yml.IsNull());
        ASSERT_THROW(YamlReader::get_scalar<int>(yml), eprosima::utils::ConfigurationException);
    }

    // sequence
    {
        Yaml yml;
        yml.push_back(3);
        yml.push_back(4);
        ASSERT_TRUE(yml.IsSequence());
        ASSERT_THROW(YamlReader::get_scalar<int>(yml), eprosima::utils::ConfigurationException);
    }

    // map
    {
        Yaml yml;
        yml["1"] = 3;
        yml["2"] = 4;
        ASSERT_TRUE(yml.IsMap());
        ASSERT_THROW(YamlReader::get_scalar<int>(yml), eprosima::utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
