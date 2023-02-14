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
        std::vector<participants::types::PortType> ports = {1, 11, 111, 11666, 11777, 65535};
        for (participants::types::PortType port : ports)
        {
            Yaml yml;
            add_field_to_yaml(
                yml,
                YamlField<participants::types::PortType>(port),
                ADDRESS_PORT_TAG);

            ASSERT_EQ(port, YamlReader::get<participants::types::PortType>(yml, ADDRESS_PORT_TAG, LATEST));
        }
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(
            YamlReader::get<participants::types::IpVersion>(yml, ADDRESS_PORT_TAG, LATEST),
            eprosima::utils::ConfigurationException);
    }

    // Incorrect format (string)
    {
        Yaml yml;
        add_field_to_yaml(
            yml,
            YamlField<std::string>("P11000"),
            ADDRESS_PORT_TAG);

        ASSERT_THROW(
            YamlReader::get<participants::types::PortType>(yml, ADDRESS_PORT_TAG, LATEST),
            eprosima::utils::ConfigurationException);
    }
}
