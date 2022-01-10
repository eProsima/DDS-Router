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

#include <ddsrouter/types/topic/Topic.hpp>

using namespace eprosima::ddsrouter;

using pair_topic_type = std::pair<std::string, std::string>;

/*
 * Return a list of non repeated random topic names
 */
std::vector<pair_topic_type> random_topic_names()
{
    return
        {
            {"topic1", "type1"},
            {"topic2", "type2"},
            {"topic3", "type3"},
            {"topic1", "type2"},
            {"topic1", "type3"},

            {"*", "*"},
            {"*rt", "*std"},
            {"rt*", "std*"},
            {"rt", "std*"},
            {"rt*", "std"},

            {".", "."},
        };
}

/*
 * Return a list of non repeated well sorted topic names
 */
std::vector<pair_topic_type> well_sorted_topic_names()
{
    return
        {
            {"*", "*"}, // Lowest element
            {"*", "a"},
            {"*", "b"},
            {"*a", "*"},

            {"Topic1", "Type1"},
            {"Topic1", "type1"},

            {"topic1", "type1"},
            {"topic1", "type2"},
            {"topic1", "type3"},
            {"topic2", "type1"},
            {"topic2", "type2"},
            {"topic2", "type3"},
            {"topic3", "type1"}, // Biggest element
        };
}

/**
 * Test Topic constructor and std getter methods
 */
TEST(TopicTest, constructor)
{
    for (pair_topic_type topic : random_topic_names())
    {
        Topic dt(topic.first, topic.second);
        ASSERT_EQ(dt.topic_name(), topic.first);
        ASSERT_EQ(dt.topic_type(), topic.second);
    }
}

/**
 * Test Topic == operator in positive cases
 */
TEST(TopicTest, equal_operator)
{
    for (pair_topic_type topic : random_topic_names())
    {
        Topic dt1(topic.first, topic.second);
        Topic dt2(topic.first, topic.second);
        ASSERT_TRUE(dt1 == dt2);
    }
}

/**
 * Test Topic < operator in positive cases
 */
TEST(TopicTest, minor_operator)
{
    std::vector<pair_topic_type> well_sorted_names = well_sorted_topic_names();

    for (int i = 0; i < well_sorted_names.size(); ++i)
    {
        // Skip same topic
        for (int j = (i + 1); j < well_sorted_names.size(); ++j)
        {
            Topic dt1(well_sorted_names[i].first, well_sorted_names[i].second);
            Topic dt2(well_sorted_names[j].first, well_sorted_names[j].second);
            ASSERT_TRUE(dt1 < dt2) << dt1 << " < " << dt2;
        }
    }
}

/**
 * Test Topic == operator in negative cases
 *
 * Test that every name in random topics is different with the others
 */
TEST(TopicTest, non_equal_operator)
{
    std::vector<pair_topic_type> names = random_topic_names();

    for (int i = 0; i < names.size(); ++i)
    {
        for (int j = 0; j < names.size(); ++j)
        {
            // Skip same topic
            if (i != j)
            {
                Topic dt1(names[i].first, names[i].second);
                Topic dt2(names[j].first, names[j].second);
                ASSERT_FALSE(dt1 == dt2);
            }
        }
    }
}

/**
 * Test Topic < operator in negative cases
 */
TEST(TopicTest, non_minor_operator)
{
    std::vector<pair_topic_type> well_sorted_names = well_sorted_topic_names();

    for (int i = 0; i < well_sorted_names.size(); ++i)
    {
        for (int j = i; j < well_sorted_names.size(); ++j)
        {
            Topic dt1(well_sorted_names[i].first, well_sorted_names[i].second);
            Topic dt2(well_sorted_names[j].first, well_sorted_names[j].second);
            ASSERT_FALSE(dt2 < dt1) << dt2 << " < " << dt1;
        }
    }
}

/**
 * Test the Topic << operator
 */
TEST(TopicTest, stdout_operator)
{
    std::vector<pair_topic_type> topics = {{"topic1", "type1"}, {"topic2", "type2"}};
    Topic dt("topic1", "type1");

    std::stringstream ss;
    ss << dt;
    ASSERT_EQ(ss.str(), "Topic{topic1;type1;no_key}");
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
