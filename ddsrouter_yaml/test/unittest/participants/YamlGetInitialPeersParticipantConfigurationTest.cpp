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

#include <ddspipe_core/testing/random_values.hpp>

#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>

#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/dds/DomainId.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

using namespace eprosima;

/**
 * Test get Participant Configuration from yaml
 * with id, kind
 *
 * Try random ids
 */
TEST(YamlGetInitialPeersParticipantConfigurationTest, get_participant_minimum)
{
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::initial_peers;
    for (uint32_t i = 0; i < ddsrouter::yaml::testing::TEST_ITERATIONS; i++)
    {
        ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id(i);
        for (uint32_t j = 0; j < ddsrouter::yaml::testing::TEST_ITERATIONS; j++)
        {
            // Create a configuration with this kind and this id
            Yaml yml;
            Yaml yml_participant;

            ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
            ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

            yml["participant"] = yml_participant;

            // Read Yaml
            ddspipe::participants::InitialPeersParticipantConfiguration result =
                    ddspipe::yaml::YamlReader::get<ddspipe::participants::InitialPeersParticipantConfiguration>(
                yml,
                "participant",
                ddspipe::yaml::YamlReaderVersion::LATEST);

            // Check result
            ASSERT_EQ(id, result.id);

            // Check default values
            ASSERT_EQ(result.connection_addresses.size(), 0u);
            ASSERT_EQ(result.listening_addresses.size(), 0u);
            ASSERT_FALSE(result.tls_configuration.is_active());
            ASSERT_EQ(
                ddspipe::participants::InitialPeersParticipantConfiguration().domain,
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
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::initial_peers;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();

    // default
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::InitialPeersParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::InitialPeersParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_FALSE(result.is_repeater);
    }

    // true
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

        // Add repeater attribute
        ddspipe::yaml::testing::repeater_to_yaml(yml_participant, true);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::InitialPeersParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::InitialPeersParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_TRUE(result.is_repeater);
    }

    // false
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

        // Add repeater attribute
        ddspipe::yaml::testing::repeater_to_yaml(yml_participant, false);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::InitialPeersParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::InitialPeersParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_FALSE(result.is_repeater);
    }

    // incorrect bool format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

        // Add incorrect repeater
        yml_participant[ddspipe::yaml::IS_REPEATER_TAG] = "ERROR";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            ddspipe::participants::InitialPeersParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::InitialPeersParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
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
