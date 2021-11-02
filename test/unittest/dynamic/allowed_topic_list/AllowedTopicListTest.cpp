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

/**********************
 * MANAGEMENT METHODS *
 **********************/

/**
 * Test \c AllowedTopicList \c clear method
 */
TEST(AllowedTopicListTest, clear)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c AllowedTopicList \c reload method
 */
TEST(AllowedTopicListTest, reload)
{
    // TODO
    ASSERT_TRUE(false);
}

/******************
 * FILTER METHODS *
 ******************/

/**
 * Test \c AllowedTopicList \c is_topic_allowed method for positive cases
 *
 * CASES:
 *  With empty list
 *  Adding simple Blacklist
 *  Adding complex Blacklist
 *  Adding simple Whitelist
 *  Adding complex Whitelist
 *  Adding simple Whitelist and Blacklist
 *  Adding complex Whitelist and Blacklist
 *  Adding simple entangled Whitelist and Blacklist
 *  Adding complex entangled Whitelist and Blacklist
 */
TEST(AllowedTopicListTest, is_topic_allowed)
{
    // With empty list
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
    // Adding simple Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding complex Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding simple Whitelist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding complex Whitelist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding simple Whitelist and Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding complex Whitelist and Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding simple entangled Whitelist and Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }

    // Adding complex entangled Whitelist and Blacklist
    {
        // TODO
        ASSERT_TRUE(false);
    }
}

/**
 * Test \c AllowedTopicList \c is_topic_allowed method for negative cases
 *  Adding simple Blacklist
 *  Adding complex Blacklist
 *  Adding full Blacklist
 *  Adding simple Whitelist
 *  Adding complex Whitelist
 *  Adding simple Whitelist and Blacklist
 *  Adding complex Whitelist and Blacklist
 *  Adding simple entangled Whitelist and Blacklist
 *  Adding complex entangled Whitelist and Blacklist
 */
TEST(AllowedTopicListTest, is_topic_not_allowed)
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
