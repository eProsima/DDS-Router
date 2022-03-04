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

#include <ddsrouter_core/types/address/Address.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

#include "../../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test read IP from yaml
 *
 * POSITIVE CASES:
 * - string values
 *
 * NEGATIVE CASES:
 * - Empty
 */
TEST(YamlGetEntityAddressTest, get_ip)
{
    // string values
    {
        std::vector<core::types::IpType> ips = {
            "1.1.1.1",
            "::1",
            "127.0.0.1",
        };

        for (core::types::IpType ip : ips)
        {
            Yaml yml;
            test::add_field_to_yaml(
                yml,
                test::YamlField<core::types::IpType>(ip),
                ADDRESS_IP_TAG);

            ASSERT_EQ(ip, YamlReader::get<core::types::IpType>(yml, ADDRESS_IP_TAG));
        }
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(YamlReader::get<core::types::IpVersion>(yml, ADDRESS_IP_VERSION_TAG),
                utils::ConfigurationException);
    }
}
