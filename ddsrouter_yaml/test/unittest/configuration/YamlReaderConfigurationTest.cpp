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

#include <iostream>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_yaml/yaml_configuration_tags.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>

using namespace eprosima;

/**
 * Test load a whole DDS Router Configuration from yaml node for v1.0 of yaml.
 *
 * CASES:
 * - trivial configuration
 */
TEST(YamlReaderConfigurationTest, ddsrouter_configuration_v1_not_supported)
{
    std::vector<const char*> yml_configurations =
    {
        // trivial configuration
        R"(
        version: v1.0
        participant1:
          type: "echo"
        participant2:
          type: "echo"
        )",
    };

    for (const char* yml_configuration : yml_configurations)
    {
        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ASSERT_THROW(
            ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml),
            utils::ConfigurationException);
    }
}

/**
 * Test load a whole DDS Router Configuration from yaml node for v2.0 of yaml.
 *
 * CASES:
 * - trivial configuration
 * - ROS common configuration
 */
TEST(YamlReaderConfigurationTest, get_ddsrouter_configuration_v2)
{
    std::vector<const char*> yml_configurations =
    {
        // trivial configuration
        R"(
        version: v2.0
        participants:
          - name: "P1"
            kind: "echo"
          - name: "P2"
            kind: "echo"
        )",

        // ROS common configuration
        R"(
        version: v2.0
        builtin:
          - name: "rt/chatter"
            type: "std_msgs::msg::dds_::String_"
        participants:
          - name: "P1"
            kind: "local"
            domain: 0
          - name: "P2"
            kind: "local"
            domain: 1
          - name: "P3"
            kind: "simple"
            domain: 2
        )",
    };

    for (const char* yml_configuration : yml_configurations)
    {
        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ddsrouter::core::DdsRouterConfiguration configuration_result =
                ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is valid
        utils::Formatter error_msg;
        ASSERT_TRUE(configuration_result.is_valid(error_msg)) << error_msg;
    }
}

/**
 * Do not set Yaml version and get default configuration
 * (currently default is v3.0)
 *
 * CASES:
 * - trivial configuration of v3.0
 */
TEST(YamlReaderConfigurationTest, get_ddsrouter_configuration_no_version)
{
    // trivial configuration of v3.0
    {
        const char* yml_configuration =
                R"(
            participants:
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ddsrouter::core::DdsRouterConfiguration configuration_result =
                ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is valid
        utils::Formatter error_msg;
        ASSERT_TRUE(configuration_result.is_valid(error_msg)) << error_msg;
    }
}

/**
 * Test error in version tag
 *
 * CASES:
 * - not existing version
 * - get wrongly defined yaml with default version (v3.0)
 */
TEST(YamlReaderConfigurationTest, version_negative_cases)
{
    // not existing version
    {
        // trivial configuration
        const char* yml_configuration =
                R"(
            version: v0.0
            participants:
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ASSERT_THROW(
            ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml),
            utils::ConfigurationException);
    }
    // trivial configuration of default version is not correct
    {
        const char* yml_configuration =
                R"(
              - name: "P1"
                kind: "echo"
              - name: "P2"
                kind: "echo"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration and check is not valid
        ASSERT_THROW(
            ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml),
            utils::ConfigurationException);
    }
}

/**
 * Test load the number of threads in the configuration
 *
 * CASES:
 * - trivial configuration
 */
TEST(YamlReaderConfigurationTest, number_of_threads)
{
    const char* yml_configuration =
            // trivial configuration
            R"(
        version: v3.0
        participants:
          - name: "P1"
            kind: "echo"
          - name: "P2"
            kind: "echo"
        )";
    Yaml yml = YAML::Load(yml_configuration);

    std::vector<unsigned int> test_cases = {1, 2, 10, 20, 42, 100};

    for (unsigned int test_case : test_cases)
    {
        Yaml yml_specs;
        yml_specs[ddspipe::yaml::NUMBER_THREADS_TAG] = test_case;
        yml[ddspipe::yaml::SPECS_TAG] = yml_specs;

        // Load configuration
        ddsrouter::core::DdsRouterConfiguration configuration_result =
                ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check threads are correct
        ASSERT_EQ(test_case, configuration_result.advanced_options.number_of_threads);
    }
}

/**
 * Test load of maximum history depth in the configuration
 *
 * CASES:
 * - trivial configuration
 */
TEST(YamlReaderConfigurationTest, max_history_depth)
{
    const char* yml_configuration =
            // trivial configuration
            R"(
        version: v3.0
        participants:
          - name: "P1"
            kind: "echo"
          - name: "P2"
            kind: "echo"
        )";
    Yaml yml = YAML::Load(yml_configuration);

    std::vector<unsigned int> test_cases = {10, 100, 1000, 5000, 10000};

    for (unsigned int test_case : test_cases)
    {
        Yaml yml_specs;
        yml_specs[ddspipe::yaml::MAX_HISTORY_DEPTH_TAG] = test_case;
        yml[ddspipe::yaml::SPECS_TAG] = yml_specs;

        // Load configuration
        ddsrouter::core::DdsRouterConfiguration configuration_result =
                ddsrouter::yaml::YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check max history depth is correct
        ASSERT_EQ(test_case, configuration_result.advanced_options.max_history_depth);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
