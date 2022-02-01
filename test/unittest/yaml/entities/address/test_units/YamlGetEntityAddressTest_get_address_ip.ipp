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

#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

#include "../../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

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
        std::vector<IpType> ips = {
            "1.1.1.1",
            "127.0.0.1",
            "8.8.8.8",
        };

        for (IpType ip : ips)
        {
            Yaml yml_address;

            // Add IP
            test::add_field_to_yaml(
                yml_address,
                test::YamlField<IpType>(ip),
                ADDRESS_IP_TAG);

            // Add IP version
            test::add_field_to_yaml(
                yml_address,
                test::YamlField<std::string>(ADDRESS_IP_VERSION_V4_TAG),
                ADDRESS_IP_VERSION_TAG);

            Yaml yml;
            yml["address"] = yml_address;

            // Get Address from Yaml
            Address result = YamlReader::get<Address>(yml, "address");

            // Check result
            ASSERT_EQ(IpVersion::IPv4, result.ip_version());
            ASSERT_EQ(ip, result.ip());
        }
    }

    // ipv6
    {
        std::vector<IpType> ips = {
            "::1",
            "12:34::89",
            "2001:0DB8:0000:0000:0000:0000:1428:57ab",
        };

        for (IpType ip : ips)
        {
            Yaml yml_address;

            // Add IP
            test::add_field_to_yaml(
                yml_address,
                test::YamlField<IpType>(ip),
                ADDRESS_IP_TAG);

            // Add IP version
            test::add_field_to_yaml(
                yml_address,
                test::YamlField<std::string>(ADDRESS_IP_VERSION_V6_TAG),
                ADDRESS_IP_VERSION_TAG);

            Yaml yml;
            yml["address"] = yml_address;

            // Get Address from Yaml
            Address result = YamlReader::get<Address>(yml, "address");

            // Check result
            ASSERT_EQ(IpVersion::IPv6, result.ip_version());
            ASSERT_EQ(ip, result.ip());
        }
    }
}
