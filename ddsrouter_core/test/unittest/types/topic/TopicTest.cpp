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

#include <ddsrouter_utils/Formatter.hpp>

#include <ddsrouter_core/types/topic/Topic.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core::types;

/*
 * Return a list of non repeated random topic names
 */
std::vector<std::string> random_topic_names()
{
    return
        {
            {"topic1"},
            {"topic2"},
            {"topic3"},

            {"*"},
            {"*rt"},
            {"rt*"},
            {"rt"},
            {"rt*"},

            {"."},
        };
}

/**
 * Test Topic constructor and std getter methods
 */
TEST(TopicTest, is_valid)
{
    utils::Formatter __f;

    for (const std::string& topic_name : random_topic_names())
    {
        Topic topic(topic_name);
        ASSERT_TRUE(topic.is_valid(__f));
    }

    ASSERT_FALSE(Topic().is_valid(__f));
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
