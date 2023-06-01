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

#include <ddspipe_core/types/participant/ParticipantId.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

constexpr const uint32_t TEST_ITERATION_MAX = 5;

using namespace eprosima;

/**
 * Test get Participant Configuration from yaml
 *
 * Try random ids with random types
 */
TEST(YamlGetCommonParticipantConfigurationTest, get_participant)
{
    for (ddsrouter::core::types::ParticipantKind kind : ddsrouter::core::types::VALUES_ParticipantKind)
    {
        for (unsigned int i = 0; i < TEST_ITERATION_MAX; i++)
        {
            ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id(i);

            // Create a configuration with this kind and this id
            Yaml yml;
            Yaml yml_participant;

            ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
            ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);

            yml["participant"] = yml_participant;

            // Read Yaml
            ddspipe::participants::ParticipantConfiguration result =
                    ddspipe::yaml::YamlReader::get<ddspipe::participants::ParticipantConfiguration>(yml, "participant",
                            ddspipe::yaml::YamlReaderVersion::LATEST);

            // Check result
            ASSERT_EQ(id, result.id);
        }
    }
}

/**
 * Test get Participant Configuration from yaml fail cases (only id is required)
 */
TEST(YamlGetCommonParticipantConfigurationTest, get_participant_negative)
{
    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        ddsrouter::yaml::testing::participantkind_to_yaml(
            yml_participant,
            ddsrouter::core::types::ParticipantKind::echo);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::ParticipantConfiguration>(yml, "participant",
            ddspipe::yaml::YamlReaderVersion::LATEST),
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
