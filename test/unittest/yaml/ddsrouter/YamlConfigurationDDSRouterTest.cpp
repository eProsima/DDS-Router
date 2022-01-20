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

#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/yaml/YamlConfigurationDDSRouter.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

#include "YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlConfigurationDDSRouterTest, get_ddsrouter_configuration_trivial)
{
    const char* yml_str = R"(
            participants:
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    configuration::DDSRouterConfiguration configuration_result =
        YamlConfigurationDDSRouter::get_ddsrouter_configuration(yml);

    // Check is valid
    ASSERT_TRUE(configuration_result.is_valid());

    // Check Topics are empty
    ASSERT_EQ(configuration_result.allowlist(), std::set<std::shared_ptr<eprosima::ddsrouter::FilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist(), std::set<std::shared_ptr<eprosima::ddsrouter::FilterTopic>>());
    ASSERT_EQ(configuration_result.builtin_topics(), std::set<std::shared_ptr<eprosima::ddsrouter::RealTopic>>());

    // Check Participant configurations
    std::set<std::shared_ptr<eprosima::ddsrouter::configuration::ParticipantConfiguration>>
        participant_configurations = configuration_result.participants_configurations();

    ASSERT_EQ(participant_configurations.size(), 2);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->type(), ParticipantKind::ECHO);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Add 3 different Participants of kind Simple and one builtin topic.
 */
TEST(YamlConfigurationDDSRouterTest, get_ddsrouter_configuration_ros_case)
{
    const char* yml_str = R"(
            builtin:
              - name: "rt/chatter"
                type: "std_msgs::msg::dds_::String_"
            participants:
              - name: "P1"
                kind: "local"
                domain: 0
              - name: "P2"
                kind: "local"
                domain: 1
              - name: "P3"
                kind: "simple"
                domain: 2
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    configuration::DDSRouterConfiguration configuration_result =
        YamlConfigurationDDSRouter::get_ddsrouter_configuration(yml);

    // Check is valid
    ASSERT_TRUE(configuration_result.is_valid());

    // Check Topic lists are empty
    ASSERT_EQ(configuration_result.allowlist(), std::set<std::shared_ptr<eprosima::ddsrouter::FilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist(), std::set<std::shared_ptr<eprosima::ddsrouter::FilterTopic>>());

    // Check Builtin Topics has one correct topic
    std::set<std::shared_ptr<eprosima::ddsrouter::RealTopic>> builtin_result = configuration_result.builtin_topics();
    ASSERT_EQ(builtin_result.size(), 1);
    std::shared_ptr<eprosima::ddsrouter::RealTopic> topic_result = (*builtin_result.begin());
    ASSERT_EQ(topic_result->topic_name(), "rt/chatter");
    ASSERT_EQ(topic_result->topic_type(), "std_msgs::msg::dds_::String_");
    ASSERT_EQ(topic_result->topic_with_key(), false);

    // Check Participant configurations
    std::set<std::shared_ptr<eprosima::ddsrouter::configuration::ParticipantConfiguration>>
        participant_configurations = configuration_result.participants_configurations();

    ASSERT_EQ(participant_configurations.size(), 3);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->type(), ParticipantKind::SIMPLE_RTPS);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
