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
#include <TestLogHandler.hpp>

#include "../../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test the behaviour when both ip and domain are given.
 *
 * The expected behaviour is for the yml to use only IP and show a warning
 */
TEST(YamlGetEntityAddressTest, ip_and_domain)
{
    // Check a warning is shown
    eprosima::ddsrouter::test::TestLogHandler log_handler(utils::Log::Kind::Warning, 1);

    // Set address
    Yaml yml_address;

    // Add domain
    yaml::test::add_field_to_yaml(
        yml_address,
        yaml::test::YamlField<core::types::IpType>("localhost"),
        ADDRESS_DNS_TAG);

    // Add ip
    core::types::IpType ip_value = "1.1.1.1";
    yaml::test::add_field_to_yaml(
        yml_address,
        yaml::test::YamlField<core::types::IpType>(ip_value),
        ADDRESS_IP_TAG);

    Yaml yml;
    yml[ADDRESS_TRANSPORT_TAG] = yml_address;

    core::types::Address address = YamlReader::get<core::types::Address>(yml, ADDRESS_TRANSPORT_TAG);

    ASSERT_EQ(address.ip(), ip_value);
}
