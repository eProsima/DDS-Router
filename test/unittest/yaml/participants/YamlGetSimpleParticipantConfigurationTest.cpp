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
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>

#include "../YamlConfigurationTestUtils.hpp"

constexpr const uint32_t TEST_ITERATION_MAX = 5;

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml
 *
 * Try random ids, random types and random domains
 */
TEST(YamlGetSimpleParticipantConfigurationTest, get_participant)
{
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        for (int i=0; i<TEST_ITERATION_MAX; i++)
        {
            ParticipantId id = eprosima::ddsrouter::test::random_participant_id(i);
            for (int j=0; j<TEST_ITERATION_MAX; j++)
            {
                DomainId domain = eprosima::ddsrouter::test::random_domain(j);

                // Create a configuration with this kind and this id
                Yaml yml;
                Yaml yml_participant;

                yaml::test::participantid_to_yaml(yml_participant, id);
                yaml::test::participantkind_to_yaml(yml_participant, kind);
                yaml::test::domain_to_yaml(yml_participant, domain);

                yml["participant"] = yml_participant;

                // Read Yaml
                configuration::SimpleParticipantConfiguration result =
                    YamlReader::get<configuration::SimpleParticipantConfiguration>(yml, "participant");

                // Check result
                ASSERT_EQ(id, result.id());
                ASSERT_EQ(kind, result.kind());
                ASSERT_EQ(domain, result.domain());
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
 * - no domain
 */
TEST(YamlGetSimpleParticipantConfigurationTest, get_participant_negative)
{
    ParticipantKind kind(ParticipantKind::SIMPLE_RTPS);
    ParticipantId id(eprosima::ddsrouter::test::random_participant_id());
    DomainId domain(17u);

    // empty
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::SimpleParticipantConfiguration result =
                YamlReader::get<configuration::SimpleParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }

    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::domain_to_yaml(yml_participant, domain);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::SimpleParticipantConfiguration result =
                YamlReader::get<configuration::SimpleParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }

    // no type
    {
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::domain_to_yaml(yml_participant, domain);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::SimpleParticipantConfiguration result =
                YamlReader::get<configuration::SimpleParticipantConfiguration>(yml, "participant"),
            ConfigurationException);
    }

    // no domain
    {
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            configuration::SimpleParticipantConfiguration result =
                YamlReader::get<configuration::SimpleParticipantConfiguration>(yml, "participant"),
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
