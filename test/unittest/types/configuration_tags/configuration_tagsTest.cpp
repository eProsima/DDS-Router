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

#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/types/configuration_tags.hpp>

using namespace eprosima::ddsrouter;

TEST(configuration_tags, ddsrouter_tags)
{
    std::set<std::string> retrieved_tags = ddsrouter_tags();

    std::set<std::string> current_tags =
    {
        ALLOWLIST_TAG,
        BLOCKLIST_TAG,
        TOPIC_NAME_TAG,
        TOPIC_TYPE_NAME_TAG
    };

    ASSERT_EQ(retrieved_tags, current_tags);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
