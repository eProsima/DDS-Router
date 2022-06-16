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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/types/topic/Topic.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

using pair_topic_type = std::pair<std::string, std::string>;

// TODO: extend contains tests for regex topics

/**
 * Test WildcardTopic construct only with topic name
 */
TEST(WildcardTopicTest, topic_name_constructor)
{
    WildcardTopic topic1("topic1");
    WildcardTopic topic2("topic1", "*");

    ASSERT_EQ(topic1, topic2);
}

/**
 * Test WildcardTopic contains method for positive cases
 */
TEST(WildcardTopicTest, contains_wildcard)
{
    std::vector<                            // Test cases
        std::pair<
            pair_topic_type,                // Wildcard Topic
            std::vector<pair_topic_type>    // List of accepted RealTopics
            >> test_cases = {

        {{"topic", "*"},
            {{"topic", "type"}, {"topic", "type*"}, {"topic", "*type"}, {"topic", "type1*"}}},

        {{"topic", "type*"},
            {{"topic", "type"}, {"topic", "type1"}, {"topic", "type1*"}}},

        {{"*", "type"},
            {{"topic", "type"}, {"*topic", "type"}, {"topic*", "type"}, {"*rt/topic", "type"}}},

        {{"*topic", "type"},
            {{"topic", "type"}, {"std_topic", "type"}, {"*rt/topic", "type"}}},

        {{"topic*", "type*"},
            {{"topic", "type"}}},
    };

    for (auto test_case : test_cases)
    {
        // Create Wildcard topic
        WildcardTopic wt(test_case.first.first, test_case.first.second);

        // For every topic to test, create a RealTopic and test
        for (auto real_topic_names : test_case.second)
        {
            WildcardTopic real_topic(real_topic_names.first, real_topic_names.second);

            ASSERT_TRUE(wt.contains(real_topic));
        }
    }
}

/**
 * Test WildcardTopic matches method for positive cases
 */
TEST(WildcardTopicTest, matches)
{
    std::vector<                            // Test cases
        std::pair<
            pair_topic_type,                // Wildcard Topic
            std::vector<pair_topic_type>    // List of accepted RealTopics
            >> test_cases = {

        {{"topic", "*"},
            {{"topic", "type"}, {"topic", "type1"}, {"topic", "type2"}}},

        {{"topic", "type*"},
            {{"topic", "type"}, {"topic", "type1"}, {"topic", "type2"}}},

        {{"*", "type"},
            {{"topic", "type"}, {"std_topic", "type"}, {"rt/topic", "type"}}},

        {{"*topic", "type"},
            {{"topic", "type"}, {"std_topic", "type"}, {"rt/topic", "type"}}},

        {{"topic", "type"},
            {{"topic", "type"}}},
    };

    for (auto test_case : test_cases)
    {
        // Create Wildcard topic
        WildcardTopic wt(test_case.first.first, test_case.first.second);

        // For every topic to test, create a RealTopic and test
        for (auto real_topic_names : test_case.second)
        {
            RealTopic real_topic(real_topic_names.first, real_topic_names.second);

            ASSERT_TRUE(wt.matches(real_topic)) << "wildcard: " << wt << " ; real: " << real_topic;
        }
    }
}

/**
 * Test WildcardTopic contains method for negative cases
 */
TEST(WildcardTopicTest, non_contains_wildcard)
{
    std::vector<                            // Test cases
        std::pair<
            pair_topic_type,                // Wildcard Topic
            std::vector<pair_topic_type>    // List of accepted RealTopics
            >> test_cases = {

        {{"topic", "*"},
            {{"topic1", "type"}, {"topic*", "type"}, {"*", "type"}, {"*topic", "*"}, {"*", "*"}}},

        {{"topic", "type*"},
            {{"topic*", "type"}, {"topic", "*type"}, {"*", "*"}}},

        {{"*", "type"},
            {{"topic", "type1"}, {"topic", "type*"}, {"topic*", "*"}, {"*", "type*"}, {"*", "*"}}},

        {{"*topic", "type"},
            {{"topic", "type*"}, {"topic*", "type"}, {"*", "*"}}},

        {{"topic*", "type*"},
            {{"*topic", "type"}, {"topic", "*type"}, {"*", "*"}}},
    };

    for (auto test_case : test_cases)
    {
        // Create Wildcard topic
        WildcardTopic wt(test_case.first.first, test_case.first.second);

        // For every topic to test, create a RealTopic and test
        for (auto real_topic_names : test_case.second)
        {
            WildcardTopic real_topic(real_topic_names.first, real_topic_names.second);

            ASSERT_FALSE(wt.contains(real_topic));
        }
    }
}

/**
 * Test WildcardTopic matches method for negative cases
 */
TEST(WildcardTopicTest, non_matches)
{
    std::vector<                            // Test cases
        std::pair<
            pair_topic_type,                // Wildcard Topic
            std::vector<pair_topic_type>    // List of accepted RealTopics
            >> test_cases = {

        {{"topic", "*"},
            {{"topic1", "type"}, {"topic1", "type1"}, {"std_topic", "type1"}}},

        {{"topic", "type*"},
            {{"topic1", "type"}, {"topic", "std::type"}, {"topic", "std::type1"}}},

        {{"*", "type"},
            {{"topic", "type1"}, {"std_topic", "type1"}, {"topic", "std::type"}}},

        {{"*topic", "type"},
            {{"topic", "type1"}, {"std_topic1", "type"}, {"rt/topic1", "type"}}},

        {{"topic", "type"},
            {{"topic", "type1"}, {"topic1", "type"}, {"topic1", "type1"}, {"std_topic", "type"}}},
    };

    for (auto test_case : test_cases)
    {
        // Create Wildcard topic
        WildcardTopic wt(test_case.first.first, test_case.first.second);

        // For every topic to test, create a RealTopic and test
        for (auto real_topic_names : test_case.second)
        {
            RealTopic real_topic(real_topic_names.first, real_topic_names.second);

            ASSERT_FALSE(wt.matches(real_topic));
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
