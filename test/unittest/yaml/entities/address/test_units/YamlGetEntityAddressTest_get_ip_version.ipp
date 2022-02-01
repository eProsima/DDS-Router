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
 * Test read Transport Protocol from yaml
 *
 * POSITIVE CASES:
 * - v4
 * - v6
 *
 * NEGATIVE CASES:
 * - Empty
 * - Incorrect value name
 * - Incorrect format
 */
TEST(YamlGetEntityAddressTest, get_ip_version)
{
    // v4
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_IP_VERSION_V4_TAG),
            ADDRESS_IP_VERSION_TAG);

        IpVersion iv = YamlReader::get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG);

        ASSERT_EQ(iv, IpVersion::IPv4);
    }

    // v6
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_IP_VERSION_V6_TAG),
            ADDRESS_IP_VERSION_TAG);

        IpVersion iv = YamlReader::get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG);

        ASSERT_EQ(iv, IpVersion::IPv6);
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(YamlReader::get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG), ConfigurationException);
    }

    // Incorrect value name
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>("v7"),
            ADDRESS_IP_VERSION_TAG);

        ASSERT_THROW(YamlReader::get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG), ConfigurationException);
    }

    // Incorrect format
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<uint32_t>(17),
            ADDRESS_IP_VERSION_TAG);

        ASSERT_THROW(YamlReader::get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG), ConfigurationException);
    }
}
