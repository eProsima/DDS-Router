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

#include <cpp_utils/testing/LogChecker.hpp>

#include <ddspipe_yaml/testing/generate_yaml.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;
using namespace eprosima::ddspipe::core::testing;
using namespace eprosima::ddspipe::yaml::testing;

/**
 * Test the behaviour when both ip and domain are given.
 *
 * The expected behaviour is for the yml to use only IP and show a warning
 */
TEST(YamlGetEntityAddressTest, ip_and_domain)
{
    // Check a warning is shown
    // eprosima::ddspipe::core::TestLogHandler log_handler(utils::Log::Kind::Warning, 1);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Warning, 1, 0);

    // Set address
    Yaml yml_address;

    // Add domain
    ddspipe::yaml::testing::add_field_to_yaml(
        yml_address,
        ddspipe::yaml::testing::YamlField<participants::types::IpType>("localhost"),
        ADDRESS_DNS_TAG);

    // Add ip
    participants::types::IpType ip_value = "1.1.1.1";
    ddspipe::yaml::testing::add_field_to_yaml(
        yml_address,
        ddspipe::yaml::testing::YamlField<participants::types::IpType>(ip_value),
        ADDRESS_IP_TAG);

    Yaml yml;
    yml[ADDRESS_TRANSPORT_TAG] = yml_address;

    participants::types::Address address = YamlReader::get<participants::types::Address>(yml, ADDRESS_TRANSPORT_TAG, LATEST);

    ASSERT_EQ(address.ip(), ip_value);
}
