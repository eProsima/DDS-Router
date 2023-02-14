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

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;

/**
 * Test get Participant Configuration from yaml with listening address
 *
 * POSITIVE CASES:
 * - 1 address
 * - N addresses
 *
 * NEGATIVE CASES:
 * - not list format
 * - incorrect address format
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_listening_addresses)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::local_discovery_server);
    core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id();
    core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();

    // 1 address
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_listening_addresses;
        Yaml yml_address;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address
        participants::types::Address address = eprosima::ddsrouter::test::random_address();
        yaml::test::address_to_yaml(yml_address, address);
        yml_listening_addresses.push_back(yml_address);

        yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::DiscoveryServerParticipantConfiguration result =
                YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_EQ(result.listening_addresses.size(), 1u);
        ASSERT_EQ(address, *result.listening_addresses.begin());
    }

    // N addresses
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_listening_addresses;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add addresses
        std::vector<participants::types::Address> addresses;
        for (int i = 0; i < eprosima::ddsrouter::test::TEST_NUMBER_ITERATIONS; i++)
        {
            // Create new address
            participants::types::Address address = eprosima::ddsrouter::test::random_address(i);
            addresses.push_back(address);

            // Add it to yaml
            Yaml yml_address;
            yaml::test::address_to_yaml(yml_address, address);
            yml_listening_addresses.push_back(yml_address);
        }

        yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::DiscoveryServerParticipantConfiguration result =
                YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_EQ(addresses.size(), result.listening_addresses.size()) << yml;
        // Check every address is inside listening addresses of the configuration
        for (participants::types::Address address : addresses)
        {
            // ATTENTION: this previous declaration is needed as listening_addresses() does not return a reference
            std::set<eprosima::ddsrouter::participants::types::Address> addresses_result = result.listening_addresses;
            ASSERT_NE(
                addresses_result.find(address),
                addresses_result.end());
        }
    }

    // not list format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address badly
        Yaml yml_listening_addresses;
        Yaml yml_address;
        participants::types::Address address = eprosima::ddsrouter::test::random_address();
        yaml::test::address_to_yaml(yml_address, address);
        yml_listening_addresses["address1"] = yml_address;
        yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            core::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            eprosima::utils::ConfigurationException);
    }

    // incorrect address format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add incorrect address
        Yaml yml_listening_addresses;
        Yaml yml_address; // empty yaml = incorrect address

        yml_listening_addresses.push_back(yml_address);
        yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            core::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            eprosima::utils::ConfigurationException);
    }
}
