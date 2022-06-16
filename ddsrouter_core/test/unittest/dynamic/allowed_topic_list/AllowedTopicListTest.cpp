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

// #include <dynamic/AllowedTopicList.hpp> TOOD anton
#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;
using namespace eprosima::ddsrouter::core::configuration;

/*
 * Create an AllowedTopicList object with the allowlist and blocklist given by argument
 * Check that all topics in real_topics_positive are allowed by the AllowedTopicList
 * Check that all topics in real_topics_negative are not allowed by the AllowedTopicList
 */
void generic_test(
        const TopicKeySet<FilterTopic>& allowlist_topics,
        const TopicKeySet<FilterTopic>& blocklist_topics,
        const TopicKeySet<RealTopic>& real_topics_positive,
        const TopicKeySet<RealTopic>& real_topics_negative)
{
    DDSRouterReloadConfiguration cfg(allowlist_topics, blocklist_topics, real_topics_positive);

    // Test positive cases
    for (const auto& topic : real_topics_positive)
    {
        ASSERT_TRUE(cfg.is_topic_allowed(topic));
    }

    // Test negative cases
    for (const auto& topic : real_topics_negative)
    {
        ASSERT_FALSE(cfg.is_topic_allowed(topic));
    }
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using default constructor
 */
TEST(AllowedTopicListTest, is_topic_allowed__default_constructor)
{
    TopicKeySet<RealTopic> real_topics =
    {
        {"topic1", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    DDSRouterReloadConfiguration cfg({}, {}, std::move(real_topics));

    for (const auto& topic : real_topics)
    {
        ASSERT_TRUE(cfg.is_topic_allowed(topic));
    }
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using empty lists
 */
TEST(AllowedTopicListTest, is_topic_allowed__empty_list)
{
    TopicKeySet<FilterTopic> allowlist_topics;
    TopicKeySet<FilterTopic> blocklist_topics;
    TopicKeySet<RealTopic> real_topics_negative;

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple blocklist: only with real topics
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_blocklist)
{
    TopicKeySet<FilterTopic> allowlist_topics;

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex blocklist: with wildcards that superpose
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_blocklist)
{
    TopicKeySet<FilterTopic> allowlist_topics;

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/chatter*", "std::*"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic2", "type2"},
        {"rt/topic_info", "std::std_msgs::string"},
        {"rt/chatter/pub", "std_type"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std::type"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple allowlist: only with real topics
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_allowlist)
{
    TopicKeySet<FilterTopic> blocklist_topics;

    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type1_"},
        {"topic1_", "type1"},
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex allowlist: with wildcards that superpose
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_allowlist)
{
    TopicKeySet<FilterTopic> blocklist_topics;

    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/chatter*", "std::*"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std::string"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/pub", "std::std_msgs::string"},
        {"rt/chatter", "std_type"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple allowlist and simple blocklist: only with real topics
 * Lists do not superpose to each other
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_allowlist_and_blocklist)
{
    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type2"},
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/pub", "std"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex allowlist and complex blocklist: with wildcards that superpose
 * Lists do not superpose to each other
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_allowlist_and_blocklist)
{
    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/pub*", "std_type::*"},
    };

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic2", "*"},
        {"*", "HelloWorldType"},
        {"rt/chatter*", "std*"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/pub/topic", "std_type::string"},
        {"rt/pub", "std_type::string"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"OtherTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple allowlist and simple blocklist: only with real topics
 * Blocklist has topics that block the allowlist ones
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_allowlist_and_blocklist_entangled)
{
    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type2"},
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/pub", "std"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex allowlist and complex blocklist: with wildcards that superpose
 * Blocklist has topics that block the allowlist ones
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_allowlist_and_blocklist_entangled)
{
    TopicKeySet<FilterTopic> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt*", "std*"},
    };

    TopicKeySet<FilterTopic> blocklist_topics =
    {
        {"topic1", "type*"},
        {"*HelloWorld", "HelloWorld"},
        {"rt/chatter*", "std_type::std_msgs*"},
    };

    TopicKeySet<RealTopic> real_topics_positive =
    {
        {"topic1", "wtype1"},
        {"topic1", "wtype2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/pub", "std_type::std_msgs::string"},
        {"rt/chatter", "std::string"},
    };

    TopicKeySet<RealTopic> real_topics_negative =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"TopicHelloWorld", "HelloWorld"},
        {"OtherHelloWorld", "HelloWorld"},
        {"rt/chatter", "std_type::std_msgs::string"},
        {"rt/chatter/pub", "std_type::std_msgs::string"},
        {"chatter", "std::std_msgs::int"},
    };

    generic_test(
        allowlist_topics,
        blocklist_topics,
        real_topics_positive,
        real_topics_negative);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
