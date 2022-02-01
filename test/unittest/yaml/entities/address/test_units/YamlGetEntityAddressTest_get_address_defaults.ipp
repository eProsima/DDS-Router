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
 * Test read an address checking that default values are being used
 *
 * POSITIVE CASES:
 * - empty
 * - with ip version ipv6
 */
TEST(YamlGetEntityAddressTest, get_address_defaults)
{
    // empty
    {
        Yaml yml_address;

        Yaml yml;
        yml["address"] = yml_address;

        // Get Address from Yaml
        ASSERT_THROW(YamlReader::get<Address>(yml, "address"), ConfigurationException);
    }

    // with ip version ipv6
    {
        Yaml yml_address;

        // Add IP with ip-version ipv6
        test::add_field_to_yaml(
            yml_address,
            test::YamlField<IpType>("::1"),
            ADDRESS_IP_TAG);

        Yaml yml;
        yml["address"] = yml_address;

        // Get Address from Yaml
        Address result = YamlReader::get<Address>(yml, "address");

        // Check result
        ASSERT_EQ("::1", result.ip());
        ASSERT_EQ(Address::default_port(), result.port());
        ASSERT_EQ(Address::default_transport_protocol(), result.transport_protocol());
        ASSERT_EQ(IpVersion::IPv6, result.ip_version());
    }
}
