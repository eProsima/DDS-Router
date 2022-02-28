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

#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>

#include "../YamlConfigurationTestUtils.hpp"

constexpr const uint32_t TEST_ITERATION_MAX = 5;

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml
 *
 * Try random ids with random types
 */
TEST(YamlGetCommonParticipantConfigurationTest, get_participant)
{
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        for (int i = 0; i < TEST_ITERATION_MAX; i++)
        {
            ParticipantId id = eprosima::ddsrouter::test::random_participant_id(i);

            // Create a configuration with this kind and this id
            Yaml yml;
            Yaml yml_participant;

            yaml::test::participantid_to_yaml(yml_participant, id);
            yaml::test::participantkind_to_yaml(yml_participant, kind);

            yml["participant"] = yml_participant;

            // Read Yaml
            configuration::ParticipantConfiguration result =
                    YamlReader::get<configuration::ParticipantConfiguration>(yml, "participant");

            // Check result
            ASSERT_EQ(id, result.id());
            ASSERT_EQ(kind, result.kind());
        }
    }
}

/**
 * Test get Participant Configuration from yaml fail cases
 *
 * NEGATIVE CASES:
 * - empty
 * - no id
 * - no type
 */
TEST(YamlGetCommonParticipantConfigurationTest, get_participant_negative)
{
    // empty
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::ParticipantConfiguration result =
            YamlReader::get<configuration::ParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }

    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantkind_to_yaml(
            yml_participant,
            ParticipantKind(ParticipantKind::ECHO));
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::ParticipantConfiguration result =
            YamlReader::get<configuration::ParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }

    // no type
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantid_to_yaml(
            yml_participant,
            eprosima::ddsrouter::test::random_participant_id());
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::ParticipantConfiguration result =
            YamlReader::get<configuration::ParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }
}
int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
