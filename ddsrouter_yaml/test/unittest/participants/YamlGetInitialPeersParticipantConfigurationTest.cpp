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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <ddsrouter_core/configuration/participant/InitialPeersParticipantConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>

#include "../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml
 * with id, kind and ds-guid
 *
 * Try random ids, random kinds and random guids
 */
TEST(YamlGetInitialPeersParticipantConfigurationTest, get_participant_minimum)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::wan);
    for (int i = 0; i < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; i++)
    {
        core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id(i);
        for (int j = 0; j < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; j++)
        {
            core::types::GuidPrefix guid = eprosima::ddsrouter::test::random_guid_prefix(j);

            // Create a configuration with this kind and this id
            Yaml yml;
            Yaml yml_participant;

            yaml::test::participantid_to_yaml(yml_participant, id);
            yaml::test::participantkind_to_yaml(yml_participant, kind);

            yml["participant"] = yml_participant;

            // Read Yaml
            core::configuration::InitialPeersParticipantConfiguration result =
                    YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(
                yml,
                "participant",
                LATEST);

            // Check result
            ASSERT_EQ(id, result.id);
            ASSERT_EQ(kind, result.kind);

            // Check default values
            ASSERT_EQ(0, result.connection_addresses.size());
            ASSERT_EQ(0, result.listening_addresses.size());
            ASSERT_FALSE(result.tls_configuration.is_active());
            ASSERT_EQ(
                core::configuration::InitialPeersParticipantConfiguration().domain,
                result.domain);
        }
    }
}

/**
 * Test get Participant Configuration from yaml specifing domain
 *
 * POSITIVE CASES:
 * - default
 * - true
 * - false
 *
 * NEGATIVE CASES:
 * - incorrect bool format
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_repeater)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::wan);
    core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id();
    core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();

    // default
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::configuration::InitialPeersParticipantConfiguration result =
                YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_FALSE(result.is_repeater);
    }

    // true
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add repeater attribute
        yaml::test::repeater_to_yaml(yml_participant, true);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::configuration::InitialPeersParticipantConfiguration result =
                YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_TRUE(result.is_repeater);
    }

    // false
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add repeater attribute
        yaml::test::repeater_to_yaml(yml_participant, false);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::configuration::InitialPeersParticipantConfiguration result =
                YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_FALSE(result.is_repeater);
    }

    // incorrect bool format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add incorrect repeater
        yml_participant[IS_REPEATER_TAG] = "ERROR";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            core::configuration::InitialPeersParticipantConfiguration result =
            YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
