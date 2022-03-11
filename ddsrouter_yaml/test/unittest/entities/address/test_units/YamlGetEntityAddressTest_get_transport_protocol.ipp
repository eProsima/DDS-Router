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
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_UDP_TAG),
            ADDRESS_TRANSPORT_TAG);

        core::types::TransportProtocol tp =
                YamlReader::get<core::types::TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG, LATEST);

        ASSERT_EQ(tp, core::types::TransportProtocol::UDP);
    }

    // TCP
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_TCP_TAG),
            ADDRESS_TRANSPORT_TAG);

        core::types::TransportProtocol tp =
                YamlReader::get<core::types::TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG, LATEST);

        ASSERT_EQ(tp, core::types::TransportProtocol::TCP);
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(
            YamlReader::get<core::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            utils::ConfigurationException);
    }

    // Incorrect tag
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>("utcp"),
            ADDRESS_TRANSPORT_TAG);

        ASSERT_THROW(
            YamlReader::get<core::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            utils::ConfigurationException);
    }

    // Incorrect format
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<uint32_t>(17),
            ADDRESS_TRANSPORT_TAG);

        ASSERT_THROW(
            YamlReader::get<core::types::TransportProtocol>(
                yml,
                ADDRESS_TRANSPORT_TAG,
                LATEST),
            utils::ConfigurationException);
    }
}
