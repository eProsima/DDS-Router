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

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>

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

    // Add addresses
    Yaml yml_listening_addresses;
    Yaml yml_address;
    core::types::Address address = eprosima::ddsrouter::test::random_address();
    yaml::test::address_to_yaml(yml_address, address);
    yml_listening_addresses.push_back(yml_address);

    for (core::types::ParticipantKind kind : core::types::ALL_VALID_PARTICIPANT_KINDS)
    {
        for (int i = 0; i < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; i++)
        {
            core::types::ParticipantName name = eprosima::ddsrouter::test::random_participant_name(i);
            core::types::ParticipantId id({name, kind});
            for (int j = 0; j < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; j++)
            {
                core::types::GuidPrefix guid = eprosima::ddsrouter::test::random_guid_prefix(j);

                // Create a configuration with this kind and this id
                Yaml yml;
                Yaml yml_participant;

                yaml::test::participantname_to_yaml(yml_participant, name);
                yaml::test::participantkind_to_yaml(yml_participant, kind);
                yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid);

                yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;

                yml["participant"] = yml_participant;

                // Read Yaml
                core::configuration::DiscoveryServerParticipantConfiguration result =
                        YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(
                    yml,
                    "participant",
                    LATEST);

                // Check result
                ASSERT_EQ(id, result.id());
                ASSERT_EQ(guid, result.discovery_server_guid_prefix());

                // Check default values
                ASSERT_EQ(0, result.connection_addresses().size());
                ASSERT_EQ(1, result.listening_addresses().size());
                ASSERT_FALSE(result.tls_configuration().is_active());
                ASSERT_EQ(
                    core::types::DEFAULT_DOMAIN_ID,
                    result.domain());
            }
        }
    }
}
