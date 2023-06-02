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

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>

#include <ddsrouter_core/types/ParticipantKind.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

using namespace eprosima;

/**
 * Test get Participant Configuration from yaml
 * with id, kind and ds-guid
 *
 * Try random ids, random kinds and random guids
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_minimum)
{
    for (ddsrouter::core::types::ParticipantKind kind : ddsrouter::core::types::VALUES_ParticipantKind)
    {
        for (uint32_t i = 0; i < ddsrouter::yaml::testing::TEST_ITERATIONS; i++)
        {
            ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id(i);
            for (uint32_t j = 0; j < ddsrouter::yaml::testing::TEST_ITERATIONS; j++)
            {
                ddspipe::core::types::GuidPrefix guid = ddspipe::core::testing::random_guid_prefix(j);

                // Create a configuration with this kind and this id
                Yaml yml;
                Yaml yml_participant;

                ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
                ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
                ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid);

                yml["participant"] = yml_participant;

                // Read Yaml
                ddspipe::participants::DiscoveryServerParticipantConfiguration result =
                        ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(
                    yml,
                    "participant",
                    ddspipe::yaml::YamlReaderVersion::LATEST);

                // Check result
                ASSERT_EQ(id, result.id);
                ASSERT_EQ(guid, result.discovery_server_guid_prefix);

                // Check default values
                ASSERT_EQ(result.connection_addresses.size(), 0u);
                ASSERT_EQ(result.listening_addresses.size(), 0u);
                ASSERT_FALSE(result.tls_configuration.is_active());
                ASSERT_EQ(
                    ddspipe::participants::DiscoveryServerParticipantConfiguration().domain,
                    result.domain);
            }
        }
    }
}
