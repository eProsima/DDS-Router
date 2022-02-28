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
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

#include "../../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test read port from yaml
 *
 * POSITIVE CASES:
 * - numeric values
 *
 * NEGATIVE CASES:
 * - Empty
 * - Incorrect format (string)
 */
TEST(YamlGetEntityAddressTest, get_port)
{
    // numeric values
    {
        std::vector<PortType> ports = {1, 11, 111, 11666, 11777, 65535};
        for (PortType port : ports)
        {
            Yaml yml;
            test::add_field_to_yaml(
                yml,
                test::YamlField<PortType>(port),
                ADDRESS_PORT_TAG);

            ASSERT_EQ(port, YamlReader::get<PortType>(yml, ADDRESS_PORT_TAG));
        }
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(YamlReader::get<IpVersion>(yml, ADDRESS_PORT_TAG), ConfigurationException);
    }

    // Incorrect format (string)
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>("P11000"),
            ADDRESS_PORT_TAG);

        ASSERT_THROW(YamlReader::get<PortType>(yml, ADDRESS_PORT_TAG), ConfigurationException);
    }
}
