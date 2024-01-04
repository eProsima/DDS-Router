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
#include <memory>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>

#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>

using namespace eprosima;

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_trivial)
{
    const char* yml_str =
            R"(
            version: v4.0
            participants:
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topics are empty
    ASSERT_EQ(configuration_result.ddspipe_configuration.allowlist,
            std::set<utils::Heritable<ddspipe::core::types::IFilterTopic>>());
    ASSERT_EQ(configuration_result.ddspipe_configuration.blocklist,
            std::set<utils::Heritable<ddspipe::core::types::IFilterTopic>>());
    ASSERT_EQ(configuration_result.ddspipe_configuration.builtin_topics,
            std::set<utils::Heritable<ddspipe::core::types::DistributedTopic>>());

    // Check Participant configurations
    std::set<std::pair<ddsrouter::core::types::ParticipantKind,
            std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 2u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant.first, ddsrouter::core::types::ParticipantKind::echo);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Add 3 different Participants of kind Simple and one builtin topic.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_ros_case)
{
    const char* yml_str =
            R"(
            version: v4.0
            builtin-topics:
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
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topic lists are empty
    ASSERT_EQ(configuration_result.ddspipe_configuration.allowlist,
            std::set<utils::Heritable<ddspipe::core::types::IFilterTopic>>());
    ASSERT_EQ(configuration_result.ddspipe_configuration.blocklist,
            std::set<utils::Heritable<ddspipe::core::types::IFilterTopic>>());

    // Check Builtin Topics has one correct topic
    ASSERT_EQ(configuration_result.ddspipe_configuration.builtin_topics.size(), 1u);

    utils::Heritable<ddspipe::core::types::DdsTopic> topic_result =
            (*configuration_result.ddspipe_configuration.builtin_topics.begin());

    ASSERT_EQ(topic_result->topic_name(), "rt/chatter");
    ASSERT_EQ(topic_result->type_name, "std_msgs::msg::dds_::String_");
    ASSERT_EQ(topic_result->topic_qos.keyed, false);
    ASSERT_EQ(topic_result->topic_qos.is_reliable(), false);

    // Check Participant configurations
    std::set<std::pair<ddsrouter::core::types::ParticipantKind,
            std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 3u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant.first, ddsrouter::core::types::ParticipantKind::simple);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
