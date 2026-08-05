// Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <filesystem>
#include <fstream>
#include <iostream>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_yaml/YamlManager.hpp>
#include <ddspipe_yaml/YamlValidator.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe::yaml;

namespace test {
// Paths and files for the tests
std::string schema_path = "./ddsrouter_config_schema.json";

// Vectors with the valid and invalid YAML files
std::vector<std::string> valid_files = {
    "./valid_config_files_router/docu_example.yaml",
    // Config files copied automatically from DDS-Router/docs/resources/getting_started
    "./valid_config_files_router/client-ddsrouter.yaml",
    "./valid_config_files_router/server-ddsrouter.yaml",
    // Config files copied automatically from DDS-Router/resources/configurations/examples
    "./valid_config_files_router/change_domain_allowlist.yaml",
    "./valid_config_files_router/change_domain.yaml",
    "./valid_config_files_router/echo.yaml",
    "./valid_config_files_router/forwarding_routes.yaml",
    "./valid_config_files_router/repeater_client.yaml",
    "./valid_config_files_router/repeater_server.yaml",
    "./valid_config_files_router/ros_discovery_client.yaml",
    "./valid_config_files_router/ros_discovery_server.yaml",
    "./valid_config_files_router/wan_client.yaml",
    "./valid_config_files_router/wan_ds_client.yaml",
    "./valid_config_files_router/wan_ds_server.yaml",
    "./valid_config_files_router/wan_server.yaml",
    "./valid_config_files_router/xml.yaml",
};

std::vector<std::string> invalid_files = {
    "./invalid_config_files_router/address_no_ip_nor_domain.yaml",
    "./invalid_config_files_router/address_no_port.yaml",
    "./invalid_config_files_router/builtin_topic_no_name.yaml",
    "./invalid_config_files_router/builtin_topic_no_type.yaml",
    "./invalid_config_files_router/ds_participant_no_discovery_server_guid.yaml",
    "./invalid_config_files_router/ds_participant_no_listening_nor_connection_addresses.yaml",
    "./invalid_config_files_router/filter_topic_no_name.yaml",
    "./invalid_config_files_router/initial_peers_no_addresses.yaml",
    "./invalid_config_files_router/invalid_version.yaml",
    "./invalid_config_files_router/no_participant_kind.yaml",
    "./invalid_config_files_router/no_participant_name.yaml",
    "./invalid_config_files_router/tls_ca_no_private_key_provided_cert.yaml",
    "./invalid_config_files_router/tls_no_ca.yaml"
};
} // namespace test

/**
 * Test that a set of valid YAML configurations pass the validation
 */
TEST(YamlValidatorDdsRouterTest, validation_passed)
{
    YamlValidator validator;
    validator.set_schema(YamlValidator::InputType::FROM_FILE, test::schema_path);

    // valid files
    {
        for (std::string st : test::valid_files)
        {
            Yaml yml = YamlManager::load_file(st);
            ASSERT_TRUE(validator.validate_YAML(yml)) << "Failed for file: " << st;
        }
    }

}

/**
 * Test that a set of invalid YAML configurations don't pass the validation
 */
TEST(YamlValidatorDdsRouterTest, validation_failed)
{
    YamlValidator validator;
    validator.set_schema(YamlValidator::InputType::FROM_FILE, test::schema_path);

    // invalid files
    {
        for (std::string st : test::invalid_files)
        {
            Yaml yml = YamlManager::load_file(st);
            // Validate is called with false to prevent filling the output with the specific errors
            ASSERT_FALSE(validator.validate_YAML(yml, false)) << "Failed for file: " << st;
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
