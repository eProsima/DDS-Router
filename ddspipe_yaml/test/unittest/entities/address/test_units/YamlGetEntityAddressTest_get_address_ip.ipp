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

#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

#include <ddspipe_yaml/testing/generate_yaml.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;
using namespace eprosima::ddspipe::core::testing;
using namespace eprosima::ddspipe::yaml::testing;

/**
 * Test read an address with IP
 *
 * POSITIVE CASES:
 * - ipv4
 * - ipv6
 */
TEST(YamlGetEntityAddressTest, get_address_ip)
{
    // ipv4
    {
        std::vector<participants::types::IpType> ips = {
            "1.1.1.1",
            "127.0.0.1",
            "8.8.8.8",
        };

        for (participants::types::IpType ip : ips)
        {
            Yaml yml_address;

            // Add IP
            add_field_to_yaml(
                yml_address,
                YamlField<participants::types::IpType>(ip),
                ADDRESS_IP_TAG);

            // Add IP version
            add_field_to_yaml(
                yml_address,
                YamlField<std::string>(ADDRESS_IP_VERSION_V4_TAG),
                ADDRESS_IP_VERSION_TAG);

            Yaml yml;
            yml["address"] = yml_address;

            // Get participants::types::Address from Yaml
            participants::types::Address result = YamlReader::get<participants::types::Address>(yml, "address", LATEST);

            // Check result
            ASSERT_EQ(participants::types::IpVersion::v4, result.ip_version());
            ASSERT_EQ(ip, result.ip());
        }
    }

    // ipv6
    {
        std::vector<participants::types::IpType> ips = {
            "::1",
            "12:34::89",
            "2001:0DB8:0000:0000:0000:0000:1428:57ab",
        };

        for (participants::types::IpType ip : ips)
        {
            Yaml yml_address;

            // Add IP
            add_field_to_yaml(
                yml_address,
                YamlField<participants::types::IpType>(ip),
                ADDRESS_IP_TAG);

            // Add IP version
            add_field_to_yaml(
                yml_address,
                YamlField<std::string>(ADDRESS_IP_VERSION_V6_TAG),
                ADDRESS_IP_VERSION_TAG);

            Yaml yml;
            yml["address"] = yml_address;

            // Get participants::types::Address from Yaml
            participants::types::Address result = YamlReader::get<participants::types::Address>(yml, "address", LATEST);

            // Check result
            ASSERT_EQ(participants::types::IpVersion::v6, result.ip_version());
            ASSERT_EQ(ip, result.ip());
        }
    }
}
