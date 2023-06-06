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
#include <ddspipe_core/types/dds/DomainId.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

constexpr const uint32_t TEST_ITERATION_MAX = 5;

using namespace eprosima;

/**
 * Test get Participant Configuration from yaml
 *
 * Try random ids, random types and random domains
 */
TEST(YamlGetSimpleParticipantConfigurationTest, get_participant)
{
    for (ddsrouter::core::types::ParticipantKind kind : ddsrouter::core::types::VALUES_ParticipantKind)
    {
        for (unsigned int i = 0; i < TEST_ITERATION_MAX; i++)
        {
            ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id(i);
            for (unsigned int j = 0; j < TEST_ITERATION_MAX; j++)
            {
                ddspipe::core::types::DomainId domain = ddspipe::core::testing::random_domain(j);

                // Create a configuration with this kind and this id
                Yaml yml;
                Yaml yml_participant;


                ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
                ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
                ddspipe::yaml::testing::domain_to_yaml(yml_participant, domain);

                yml["participant"] = yml_participant;

                // Read Yaml
                ddspipe::participants::SimpleParticipantConfiguration result =
                        ddspipe::yaml::YamlReader::get<ddspipe::participants::SimpleParticipantConfiguration>(yml,
                                "participant",
                                ddspipe::yaml::YamlReaderVersion::LATEST);

                // Check result
                ASSERT_EQ(id, result.id);
                ASSERT_EQ(domain, result.domain);
            }
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
TEST(YamlGetSimpleParticipantConfigurationTest, get_participant_negative)
{
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::simple;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();
    ddspipe::core::types::DomainId domain;

    // empty
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::SimpleParticipantConfiguration>(yml, "participant",
            ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::domain_to_yaml(yml_participant, domain);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::SimpleParticipantConfiguration>(yml, "participant",
            ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // no type
    {
        Yaml yml;
        Yaml yml_participant;
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::domain_to_yaml(yml_participant, domain);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::SimpleParticipantConfiguration>(yml, "participant",
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
