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

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

using namespace eprosima;

/**
 * Test get Participant Configuration from yaml with connection address
 *
 * POSITIVE CASES:
 * - 1 address
 * - 1 connection N addresses
 * - N connections 1 addresses
 *
 * NEGATIVE CASES:
 * - not list format
 * - incorrect guid format
 * - not list of addresses
 * - incorrect address format
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_connection_addresses)
{
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::discovery_server;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();
    ddspipe::core::types::GuidPrefix guid_prefix = ddspipe::core::testing::random_guid_prefix();

    // 1 address
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;
        Yaml yml_address;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address
        ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address();
        ddspipe::yaml::testing::address_to_yaml(yml_address, address);
        yml_addresses.push_back(yml_address);
        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Add server guid
        ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix();
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

        yml_connection_addresses.push_back(yml_connection_address);

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::DiscoveryServerParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_EQ(result.connection_addresses.size(), 1u);
        ASSERT_EQ(result.connection_addresses.begin()->addresses().size(), 1u);
        ASSERT_EQ(connection_guid, result.connection_addresses.begin()->discovery_server_guid_prefix());
        ASSERT_EQ(address, *result.connection_addresses.begin()->addresses().begin());
    }

    // 1 connection N addresses
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add addresses
        std::vector<ddspipe::participants::types::Address> addresses;
        for (uint32_t i = 0; i < ddsrouter::yaml::testing::TEST_ITERATIONS; i++)
        {
            // Create new address
            ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address(i);
            addresses.push_back(address);

            // Add it to yaml
            Yaml yml_address;
            ddspipe::yaml::testing::address_to_yaml(yml_address, address);
            yml_addresses.push_back(yml_address);
        }
        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Add server guid
        ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix();
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

        yml_connection_addresses.push_back(yml_connection_address);

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::DiscoveryServerParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_EQ(result.connection_addresses.size(), 1u);
        ASSERT_EQ(addresses.size(), result.connection_addresses.begin()->addresses().size());
        ASSERT_EQ(connection_guid, result.connection_addresses.begin()->discovery_server_guid_prefix());
        // Check every address is inside connection addresses of the configuration
        for (ddspipe::participants::types::Address address : addresses)
        {
            // ATTENTION: this previous declaration is needed as listening_addresses() does not return a reference
            std::set<ddspipe::participants::types::Address> addresses_result =
                    result.connection_addresses.begin()->addresses();
            ASSERT_NE(
                addresses_result.find(address),
                addresses_result.end());
        }
    }

    // N connections 1 address
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        std::vector<ddspipe::core::types::GuidPrefix> connection_guids;

        // Add connections
        for (uint32_t i = 0; i < ddsrouter::yaml::testing::TEST_ITERATIONS; i++)
        {
            Yaml yml_connection_address;
            Yaml yml_addresses;

            // Add address
            ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address(i);

            // Add it to yaml
            Yaml yml_address;
            ddspipe::yaml::testing::address_to_yaml(yml_address, address);
            yml_addresses.push_back(yml_address);
            yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

            // Add server guid
            ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix(i);
            connection_guids.push_back(connection_guid);
            ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

            yml_connection_addresses.push_back(yml_connection_address);
        }


        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::DiscoveryServerParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_EQ(connection_guids.size(), result.connection_addresses.size());
        // Check that every connection has a correct number of addresses and a guid given
        for (ddspipe::participants::types::DiscoveryServerConnectionAddress connection : result.connection_addresses)
        {
            ASSERT_NE(
                std::find(connection_guids.begin(), connection_guids.end(), connection.discovery_server_guid_prefix())
                , connection_guids.end());
            ASSERT_EQ(connection.addresses().size(), 1u);
        }
    }

    // not list format
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;
        Yaml yml_address;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address
        ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address();
        ddspipe::yaml::testing::address_to_yaml(yml_address, address);
        yml_addresses.push_back(yml_address);
        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Add server guid
        ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix();
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

        // Add connection wrongly
        yml_connection_addresses["address1"] = yml_connection_address;

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            ddspipe::participants::DiscoveryServerParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // incorrect guid format
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;
        Yaml yml_address;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address
        ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address();
        ddspipe::yaml::testing::address_to_yaml(yml_address, address);
        yml_addresses.push_back(yml_address);
        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Do not add guid server
        // Add incorrect connection wrongly
        yml_connection_addresses.push_back(yml_connection_address);

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            ddspipe::participants::DiscoveryServerParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // not list of addresses
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;
        Yaml yml_address;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add address
        ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address();
        ddspipe::yaml::testing::address_to_yaml(yml_address, address);
        // Add address wrongly
        yml_addresses["address1"] = yml_address;

        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Add server guid
        ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix();
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

        // Add connection
        yml_connection_addresses.push_back(yml_connection_address);

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            ddspipe::participants::DiscoveryServerParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // incorrect address format
    {
        Yaml yml;
        Yaml yml_participant;
        Yaml yml_connection_addresses;
        Yaml yml_connection_address;
        Yaml yml_addresses;
        Yaml yml_address;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add empty address
        yml_addresses.push_back(yml_address);
        yml_connection_address[ddspipe::yaml::COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Add server guid
        ddspipe::core::types::GuidPrefix connection_guid = ddspipe::core::testing::random_guid_prefix();
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_connection_address, connection_guid);

        // Add connection wrongly
        yml_connection_addresses.push_back(yml_connection_address);

        yml_participant[ddspipe::yaml::CONNECTION_ADDRESSES_TAG] = yml_connection_addresses;
        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            ddspipe::participants::DiscoveryServerParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }
}
