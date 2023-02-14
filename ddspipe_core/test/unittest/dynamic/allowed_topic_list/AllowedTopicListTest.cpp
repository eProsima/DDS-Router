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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/dynamic/AllowedTopicList.hpp>
#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe::core;
using namespace eprosima::ddspipe::core::types;

using pair_topic_type = std::pair<std::string, std::string>;

namespace test
{

/******************
* FILTER METHODS *
******************/

/*
 * Add a topic to a list
 *
 * TODO: Add regex when implemented
 */
void add_topic_to_list(
        std::set<utils::Heritable<IFilterTopic>>& list,
        pair_topic_type topic_name,
        bool wildcard = true)
{
    if (wildcard)
    {
        auto new_topic = utils::Heritable<WildcardDdsFilterTopic>::make_heritable();
        new_topic->topic_name = topic_name.first;
        new_topic->type_name = topic_name.second;

        list.insert(new_topic);
    }
}

/*
 * Add several topic to a list
 *
 * TODO: Add regex when implemented
 */
void add_topics_to_list(
        std::set<utils::Heritable<IFilterTopic>>& list,
        std::vector<pair_topic_type> topic_names,
        bool wildcard = true)
{
    if (wildcard)
    {
        for (pair_topic_type topic_name : topic_names)
        {
            auto new_topic = utils::Heritable<WildcardDdsFilterTopic>::make_heritable();
            new_topic->topic_name = topic_name.first;
            new_topic->type_name = topic_name.second;

            list.insert(new_topic);
        }
    }
}

/*
 * Create an AllowedTopicList object with the allowlist and blocklist given by argument
 * Check that all topics in real_topics_positive are allowed by the AllowedTopicList
 * Check that all topics in real_topics_negative are not allowed by the AllowedTopicList
 */
void generic_test(
        const std::vector<pair_topic_type>& allowlist_topics,
        const std::vector<pair_topic_type>& blocklist_topics,
        const std::vector<pair_topic_type>& real_topics_positive,
        const std::vector<pair_topic_type>& real_topics_negative)
{
    // Create AllowedTopicList object
    std::set<utils::Heritable<IFilterTopic>> allowlist;
    std::set<utils::Heritable<IFilterTopic>> blocklist;

    add_topics_to_list(allowlist, allowlist_topics);
    add_topics_to_list(blocklist, blocklist_topics);

    AllowedTopicList atl(allowlist, blocklist);

    // Test positive cases
    for (pair_topic_type topic_name : real_topics_positive)
    {
        DdsTopic topic;
        topic.m_topic_name = topic_name.first;
        topic.type_name = topic_name.second;
        ASSERT_TRUE(atl.is_topic_allowed(topic));
    }

    // Test negative cases
    for (pair_topic_type topic_name : real_topics_negative)
    {
        DdsTopic topic;
        topic.m_topic_name = topic_name.first;
        topic.type_name = topic_name.second;
        ASSERT_FALSE(atl.is_topic_allowed(topic));
    }
}

} // test

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using default constructor
 */
TEST(AllowedTopicListTest, is_topic_allowed__default_constructor)
{
    AllowedTopicList atl;

    std::vector<pair_topic_type> real_topics =
    {
        {"topic1", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    for (pair_topic_type topic_name : real_topics)
    {
        DdsTopic topic;
        topic.m_topic_name = topic_name.first;
        topic.type_name = topic_name.second;

        ASSERT_TRUE(atl.is_topic_allowed(topic));
    }
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using empty lists
 */
TEST(AllowedTopicListTest, is_topic_allowed__empty_list)
{
    std::vector<pair_topic_type> allowlist_topics;
    std::vector<pair_topic_type> blocklist_topics;
    std::vector<pair_topic_type> real_topics_negative;

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics;

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics;

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/chatter*", "std::*"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic2", "type2"},
        {"rt/topic_info", "std::std_msgs::string"},
        {"rt/chatter/pub", "std_type"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std::type"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> blocklist_topics;

    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type1_"},
        {"topic1_", "type1"},
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> blocklist_topics;

    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/chatter*", "std::*"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std::string"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/pub", "std::std_msgs::string"},
        {"rt/chatter", "std_type"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type2"},
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/pub", "std"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt/pub*", "std_type::*"},
    };

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic2", "*"},
        {"*", "HelloWorldType"},
        {"rt/chatter*", "std*"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/pub/topic", "std_type::string"},
        {"rt/pub", "std_type::string"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"OtherTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/chatter/pub", "std"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic2", "type2"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "type1"},
        {"HelloWorldTopic", "HelloWorld"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type2"},
        {"topic2", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorldType"},
        {"rt/chatter", "std::std_msgs::string"},
        {"rt/pub", "std"},
    };

    test::generic_test(
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
    std::vector<pair_topic_type> allowlist_topics =
    {
        {"topic1", "*"},
        {"*", "HelloWorld"},
        {"rt*", "std*"},
    };

    std::vector<pair_topic_type> blocklist_topics =
    {
        {"topic1", "type*"},
        {"*HelloWorld", "HelloWorld"},
        {"rt/chatter*", "std_type::std_msgs*"},
    };

    std::vector<pair_topic_type> real_topics_positive =
    {
        {"topic1", "wtype1"},
        {"topic1", "wtype2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"OtherTopic", "HelloWorld"},
        {"rt/pub", "std_type::std_msgs::string"},
        {"rt/chatter", "std::string"},
    };

    std::vector<pair_topic_type> real_topics_negative =
    {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"TopicHelloWorld", "HelloWorld"},
        {"OtherHelloWorld", "HelloWorld"},
        {"rt/chatter", "std_type::std_msgs::string"},
        {"rt/chatter/pub", "std_type::std_msgs::string"},
        {"chatter", "std::std_msgs::int"},
    };

    test::generic_test(
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
