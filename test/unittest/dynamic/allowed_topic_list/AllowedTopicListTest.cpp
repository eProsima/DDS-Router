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

#include <databroker/dynamic/AllowedTopicList.hpp>
#include <databroker/types/topic/WildcardTopic.hpp>

using namespace eprosima::databroker;

using pair_topic_type = std::pair<std::string, std::string>;

/******************
 * FILTER METHODS *
 ******************/

/**
 *  Adding simple Whitelist and Blacklist
 *  Adding complex Whitelist and Blacklist
 *  Adding simple entangled Whitelist and Blacklist
 *  Adding complex entangled Whitelist and Blacklist
 */

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
        RealTopic topic(topic_name.first, topic_name.second);

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
    std::list<std::shared_ptr<AbstractTopic>> whitelist;
    std::list<std::shared_ptr<AbstractTopic>> blacklist;

    AllowedTopicList atl(whitelist, blacklist);

    std::vector<pair_topic_type> real_topics =
    {
        {"topic1", "type1"},
        {"topic2", "type2"},
        {"HelloWorldTopic", "HelloWorld"},
        {"rt/chatter", "std::std_msgs::string"},
    };

    for (pair_topic_type topic_name : real_topics)
    {
        RealTopic topic(topic_name.first, topic_name.second);

        ASSERT_TRUE(atl.is_topic_allowed(topic));
    }
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple blacklist: only with real topics
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_blacklist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex blacklist: with wildcards that superpose
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_blacklist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple whitelist: only with real topics
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_whitelist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex whitelist: with wildcards that superpose
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_whitelist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple whitelist and simple blacklist: only with real topics
 * Lists do not superpose to each other
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_whitelist_and_blacklist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex whitelist and complex blacklist: with wildcards that superpose
 * Lists do not superpose to each other
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_whitelist_and_blacklist)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using simple whitelist and simple blacklist: only with real topics
 * Blacklist has topics that block the whitelist ones
 */
TEST(AllowedTopicListTest, is_topic_allowed__simple_whitelist_and_blacklist_entangled)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method
 *
 * Case using complex whitelist and complex blacklist: with wildcards that superpose
 * Blacklist has topics that block the whitelist ones
 */
TEST(AllowedTopicListTest, is_topic_allowed__complex_whitelist_and_blacklist_entangled)
{
    // TODO
    ASSERT_TRUE(false);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
