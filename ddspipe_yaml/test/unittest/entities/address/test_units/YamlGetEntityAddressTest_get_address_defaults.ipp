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

        // Get participants::types::Address from Yaml
        ASSERT_THROW(YamlReader::get<participants::types::Address>(yml, "address",
                LATEST), eprosima::utils::ConfigurationException);
    }

    // with ip version ipv6
    {
        Yaml yml_address;

        // Add IP with ip-version ipv6
        add_field_to_yaml(
            yml_address,
            YamlField<participants::types::IpType>("::1"),
            ADDRESS_IP_TAG);

        Yaml yml;
        yml["address"] = yml_address;

        // Get participants::types::Address from Yaml
        participants::types::Address result = YamlReader::get<participants::types::Address>(yml, "address", LATEST);

        // Check result
        ASSERT_EQ("::1", result.ip());
        ASSERT_EQ(participants::types::Address::default_port(), result.port());
        ASSERT_EQ(participants::types::Address::default_port(), result.external_port());
        ASSERT_EQ(participants::types::Address::default_transport_protocol(), result.transport_protocol());
        ASSERT_EQ(participants::types::IpVersion::v6, result.ip_version());
    }
}
