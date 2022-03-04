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

#include <ddsrouter_core/types/topic/RealTopic.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

using pair_topic_type = std::pair<std::string, std::string>;

/**
 * Test RealTopic is_real_topic method for positive cases
 */
TEST(RealTopicTest, is_real_topic)
{
    std::vector<pair_topic_type> topics = {
        {"topic1", "type1"},
        {"topic1", "type2"},
        {"topic2", "type1"},

        {"rt/chatter", "std::std_msg::string"},

        {"HelloWorldTopic", "HelloWorld"},
    };

    for (pair_topic_type topic : topics)
    {
        ASSERT_TRUE(RealTopic::is_real_topic(topic.first, topic.second));
    }
}

/**
 * Test RealTopic is_real_topic method for negative cases
 */
TEST(RealTopicTest, is_non_real_topic)
{
    std::vector<pair_topic_type> topics = {
        {"topic", "type*"},
        {"topic*", "type"},
        {"topic*", "type*"},

        {"topic", "*type"},
        {"*topic", "type"},
        {"*topic", "*type"},

        {"topic", "*type*"},
        {"*topic*", "type"},
        {"*topic*", "*type*"},

        {"topic", "ty*pe"},
        {"top*ic", "type"},
        {"top*ic", "ty*pe"},

        {"*", "type"},
        {"topic", "*"},
        {"*", "*"},

        // TODO add regex cases
    };

    for (pair_topic_type topic : topics)
    {
        ASSERT_FALSE(RealTopic::is_real_topic(topic.first, topic.second));
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
