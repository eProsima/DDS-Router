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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

using namespace eprosima;

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_trivial)
{
    const char* yml_str =
            R"(
            version: v3.0
            participants:
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddspipe::yaml::YamlReader::get<core::DdsRouterConfiguration>(yml, ddspipe::yaml::YamlReaderVersion::LATEST);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topics are empty
    ASSERT_EQ(configuration_result.allowlist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.builtin_topics, std::set<std::shared_ptr<ddspipe::core::types::DdsTopic>>());

    // Check Participant configurations
    std::set<std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 2u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->kind, ddsrouter::core::types::ParticipantKind::echo);
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
            version: v3.0
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
            ddspipe::yaml::YamlReader::get<core::DdsRouterConfiguration>(yml, ddspipe::yaml::YamlReaderVersion::V_2_0);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topic lists are empty
    ASSERT_EQ(configuration_result.allowlist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());

    // Check Builtin Topics has one correct topic
    std::set<std::shared_ptr<ddspipe::core::types::DdsTopic>> builtin_result = configuration_result.builtin_topics;
    ASSERT_EQ(builtin_result.size(), 1u);
    std::shared_ptr<ddspipe::core::types::DdsTopic> topic_result = (*builtin_result.begin());
    ASSERT_EQ(topic_result->topic_name, "rt/chatter");
    ASSERT_EQ(topic_result->type_name, "std_msgs::msg::dds_::String_");
    ASSERT_EQ(topic_result->keyed, false);
    ASSERT_EQ(topic_result->topic_qos.get_reference().is_reliable(), false);

    // Check Participant configurations
    std::set<std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 3u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->kind, ddsrouter::core::types::ParticipantKind::simple_rtps);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_trivial_v1)
{
    const char* yml_str =
            R"(
            version: v1.0
            participant1:
              type: "echo"
            participant2:
              type: "echo"
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddspipe::yaml::YamlReader::get<core::DdsRouterConfiguration>(yml, ddspipe::yaml::YamlReaderVersion::V_1_0);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topics are empty
    ASSERT_EQ(configuration_result.allowlist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.builtin_topics, std::set<std::shared_ptr<ddspipe::core::types::DdsTopic>>());

    // Check Participant configurations
    std::set<std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 2u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->kind, ddsrouter::core::types::ParticipantKind::echo);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_builtin_v1)
{
    const char* yml_str =
            R"(
            version: v1.0
            allowlist:
              - name: "topic1"
                type: "type1"
              - name: "topic2"
                type: "*"
            participant1:
              type: "echo"
            participant2:
              type: "echo"
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddspipe::yaml::YamlReader::get<core::DdsRouterConfiguration>(yml, ddspipe::yaml::YamlReaderVersion::V_1_0);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check block Topics are empty
    ASSERT_EQ(configuration_result.blocklist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());

    // Check allowlist has 2 topics
    std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>> allowlist_result = configuration_result.allowlist;
    ASSERT_EQ(allowlist_result.size(), 2u);

    // Check Builtin Topics has one correct topic
    std::set<std::shared_ptr<ddspipe::core::types::DdsTopic>> builtin_result = configuration_result.builtin_topics;
    ASSERT_EQ(builtin_result.size(), 1u);
    std::shared_ptr<ddspipe::core::types::DdsTopic> topic_result = (*builtin_result.begin());
    ASSERT_EQ(topic_result->topic_name, "topic1");
    ASSERT_EQ(topic_result->type_name, "type1");
    ASSERT_EQ(topic_result->keyed, false);

    // Check Participant configurations
    std::set<std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 2u);

    for (auto participant : participant_configurations)
    {
        ASSERT_EQ(participant->kind, ddsrouter::core::types::ParticipantKind::echo);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node.
 * Only set two echo participants.
 */
TEST(YamlGetConfigurationDdsRouterTest, get_ddsrouter_configuration_discovery_server_v1)
{
    const char* yml_str =
            R"(
            version: v1.0
            participant1:
              type: "echo"
            participant2:
              type: "discovery-server"
              id: 3
              ros-discovery-server: true
              connection-addresses:
                - guid: "01.0f.00.00.00.00.00.00.00.00.00.00"
                  addresses:
                    - ip: 127.0.0.1
                      port: 3333
        )";

    Yaml yml = YAML::Load(yml_str);

    // Load configuration
    ddsrouter::core::DdsRouterConfiguration configuration_result =
            ddspipe::yaml::YamlReader::get<core::DdsRouterConfiguration>(yml, ddspipe::yaml::YamlReaderVersion::V_1_0);

    // Check is valid
    utils::Formatter error_msg;
    ASSERT_TRUE(configuration_result.is_valid(error_msg));

    // Check Topics are empty
    ASSERT_EQ(configuration_result.allowlist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.blocklist, std::set<std::shared_ptr<ddspipe::core::types::DdsFilterTopic>>());
    ASSERT_EQ(configuration_result.builtin_topics, std::set<std::shared_ptr<ddspipe::core::types::DdsTopic>>());

    // Check Participant configurations
    std::set<std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
    participant_configurations = configuration_result.participants_configurations;

    ASSERT_EQ(participant_configurations.size(), 2u);

    for (std::shared_ptr<ddspipe::participants::ParticipantConfiguration> participant : participant_configurations)
    {
        // If it is not the discovery server participant, continue
        if (!(participant->id() == "participant1"))
        {
            continue;
        }
        else
        {
            // Check the DS partipant is correct
            std::shared_ptr<ddspipe::participants::DiscoveryServerParticipantConfiguration> ds_participant =
                    std::dynamic_pointer_cast<ddspipe::participants::DiscoveryServerParticipantConfiguration>(participant);

            // Check Name
            ASSERT_EQ(ds_participant->id, ddspipe::core::types::ParticipantId("participant2"));

            // Check GuidPrefix
            ASSERT_EQ(
                ds_participant->discovery_server_guid_prefix,
                ddspipe::core::types::GuidPrefix(true, 3u));

            // Check Connection addresses
            ASSERT_EQ(
                ds_participant->connection_addresses.size(),
                1u);
            ddspipe::participants::types::DiscoveryServerConnectionAddress address = *ds_participant->connection_addresses.begin();
            ASSERT_EQ(
                address,
                (ddspipe::participants::types::DiscoveryServerConnectionAddress(
                    ddspipe::core::types::GuidPrefix("01.0f.00.00.00.00.00.00.00.00.00.00"),
                    {ddspipe::participants::types::Address("127.0.0.1", 3333, 3333, ddspipe::participants::types::Address::default_transport_protocol())}
                    ))
                );
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
