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

#include "../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml
 * with id, kind and ds-guid
 *
 * Try random ids, random kinds and random guids
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_minimum)
{
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        for (int i = 0; i < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; i++)
        {
            ParticipantId id = eprosima::ddsrouter::test::random_participant_id(i);
            for (int j = 0; j < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; j++)
            {
                GuidPrefix guid = eprosima::ddsrouter::test::random_guid_prefix(j);

                // Create a configuration with this kind and this id
                Yaml yml;
                Yaml yml_participant;

                yaml::test::participantid_to_yaml(yml_participant, id);
                yaml::test::participantkind_to_yaml(yml_participant, kind);
                yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid);

                yml["participant"] = yml_participant;

                // Read Yaml
                configuration::DiscoveryServerParticipantConfiguration result =
                        YamlReader::get<configuration::DiscoveryServerParticipantConfiguration>(yml, "participant");

                // Check result
                ASSERT_EQ(id, result.id());
                ASSERT_EQ(kind, result.kind());
                ASSERT_EQ(guid, result.discovery_server_guid_prefix());

                // Check default values
                ASSERT_EQ(0, result.connection_addresses().size());
                ASSERT_EQ(0, result.listening_addresses().size());
                ASSERT_FALSE(result.tls_active());
                ASSERT_EQ(configuration::DiscoveryServerParticipantConfiguration::default_domain_id(), result.domain());
            }
        }
    }
}