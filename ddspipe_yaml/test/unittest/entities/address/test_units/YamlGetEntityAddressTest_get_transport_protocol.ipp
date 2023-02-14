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
 * Test read Transport Protocol from yaml
 *
 * POSITIVE CASES:
 * - UDP
 * - TCP
 *
 * NEGATIVE CASES:
 * - Empty
 * - Incorrect tag
 */
TEST(YamlGetEntityAddressTest, get_transport_protocol)
{
    // UDP
    {
        Yaml yml;
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_TRANSPORT_UDP_TAG),
            ADDRESS_TRANSPORT_TAG);

        participants::types::TransportProtocol tp =
                YamlReader::get<participants::types::TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG, LATEST);

        ASSERT_EQ(tp, participants::types::TransportProtocol::udp);
    }

    // TCP
    {
        Yaml yml;
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_TRANSPORT_TCP_TAG),
            ADDRESS_TRANSPORT_TAG);

        participants::types::TransportProtocol tp =
                YamlReader::get<participants::types::TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG, LATEST);

        ASSERT_EQ(tp, participants::types::TransportProtocol::tcp);
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(
            YamlReader::get<participants::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            eprosima::utils::ConfigurationException);
    }

    // Incorrect tag
    {
        Yaml yml;
        add_field_to_yaml(
            yml,
            YamlField<std::string>("utcp"),
            ADDRESS_TRANSPORT_TAG);

        ASSERT_THROW(
            YamlReader::get<participants::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            eprosima::utils::ConfigurationException);
    }

    // Incorrect format
    {
        Yaml yml;
        add_field_to_yaml(
            yml,
            YamlField<uint32_t>(17),
            ADDRESS_TRANSPORT_TAG);

        ASSERT_THROW(
            YamlReader::get<participants::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            eprosima::utils::ConfigurationException);
    }
}
