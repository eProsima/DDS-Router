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

#include <ddsrouter_core/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

#include "../../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test read a discovery server connection address that fails
 *
 * NEGATIVE CASES:
 * - empty
 * - no guid prefix
 * - no addresses
 * - error format in guid prefix
 * - error format in addresses
 */
TEST(YamlGetEntityDiscoveryServerAddressTest, get_ds_address_negative)
{
    // empty
    {
        Yaml yml_empty;
        Yaml yml;
        yml["connection-address"] = yml_empty;

        ASSERT_THROW(
            YamlReader::get<core::types::DiscoveryServerConnectionAddress>(yml, "connection-address"),
            utils::ConfigurationException);
    }

    // no guid prefix
    {
        Yaml yml_ds_address;

        // Get random address and add it to yaml
        core::types::Address address = eprosima::ddsrouter::test::random_address();
        Yaml yml_addresses;
        Yaml yml_address;
        yaml::test::address_to_yaml(yml_address, address);
        yml_addresses.push_back(yml_address);
        yml_ds_address[COLLECTION_ADDRESSES_TAG] = yml_addresses;

        Yaml yml;
        yml["connection-address"] = yml_ds_address;
        ASSERT_THROW(
            YamlReader::get<core::types::DiscoveryServerConnectionAddress>(yml, "connection-address"),
            utils::ConfigurationException);
    }

    // no addresses
    {
        Yaml yml_ds_address;

        // Get random guid prefix and add it to yaml
        core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();
        Yaml yml_guid;
        yaml::test::guid_prefix_to_yaml(yml_guid, guid_prefix);
        yml_ds_address[DISCOVERY_SERVER_GUID_PREFIX_TAG] = yml_guid;

        Yaml yml;
        yml["connection-address"] = yml_ds_address;
        ASSERT_THROW(
            YamlReader::get<core::types::DiscoveryServerConnectionAddress>(yml, "connection-address"),
            utils::ConfigurationException);
    }

    // error format in guid prefix
    {
        Yaml yml_ds_address;

        // Get random address and add it to yaml
        core::types::Address address = eprosima::ddsrouter::test::random_address();
        Yaml yml_addresses;
        Yaml yml_address;
        yaml::test::address_to_yaml(yml_address, address);
        yml_addresses.push_back(yml_address);
        yml_ds_address[COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Guid Prefix error format (inside a sequence)
        core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();
        Yaml yml_guid;
        yaml::test::guid_prefix_to_yaml(yml_guid, guid_prefix);
        Yaml yml_guid_aux;
        yml_guid_aux.push_back(yml_guid);
        yml_ds_address[DISCOVERY_SERVER_GUID_PREFIX_TAG] = yml_guid_aux;

        Yaml yml;
        yml["connection-address"] = yml_ds_address;
        ASSERT_THROW(
            YamlReader::get<core::types::DiscoveryServerConnectionAddress>(yml, "connection-address"),
            utils::ConfigurationException);
    }

    // error format in addresses
    {
        Yaml yml_ds_address;

        // Address error (in map instead of sequence)
        core::types::Address address = eprosima::ddsrouter::test::random_address();
        Yaml yml_addresses;
        Yaml yml_address;
        yaml::test::address_to_yaml(yml_address, address);
        yml_addresses["address1"] = yml_address;
        yml_ds_address[COLLECTION_ADDRESSES_TAG] = yml_addresses;

        // Get random guid prefix and add it to yaml
        core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();
        Yaml yml_guid;
        yaml::test::guid_prefix_to_yaml(yml_guid, guid_prefix);
        yml_ds_address[DISCOVERY_SERVER_GUID_PREFIX_TAG] = yml_guid;

        Yaml yml;
        yml["connection-address"] = yml_ds_address;
        ASSERT_THROW(
            YamlReader::get<core::types::DiscoveryServerConnectionAddress>(yml, "connection-address"),
            utils::ConfigurationException);
    }
}
