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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <ddsrouter_core/participants/participant/configuration/InitialPeersParticipantConfiguration.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>

#include "../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml
 * with id, kind
 *
 * Try random ids
 */
TEST(YamlGetInitialPeersParticipantConfigurationTest, get_participant_minimum)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::wan_initial_peers);
    for (int i = 0; i < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; i++)
    {
        core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id(i);
        for (int j = 0; j < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; j++)
        {
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
            ASSERT_EQ(result.connection_addresses.size(), 0u);
            ASSERT_EQ(result.listening_addresses.size(), 0u);
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
TEST(YamlGetInitialPeersParticipantConfigurationTest, get_participant_repeater)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::wan_initial_peers);
    core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id();

    // default
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);

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

        // Add incorrect repeater
        yml_participant[IS_REPEATER_TAG] = "ERROR";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            core::configuration::InitialPeersParticipantConfiguration result =
            YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, "participant", LATEST),
            eprosima::utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
