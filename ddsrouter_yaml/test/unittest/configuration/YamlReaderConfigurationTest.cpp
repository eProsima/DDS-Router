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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

#include "../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test load a whole DDS Router Configuration from yaml node for v1.0 of yaml.
 *
 * CASES:
 * - trivial configuration
 */
TEST(YamlReaderConfigurationTest, get_ddsrouter_configuration_v1)
{
    std::vector<const char*> yml_configurations =
    {
        // trivial configuration
        R"(
        version: v1.0
        participant1:
          type: "void"
        participant2:
          type: "void"
        )",
    };

    for (const char* yml_configuration : yml_configurations)
    {
        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is valid
        utils::Formatter error_msg;
        ASSERT_TRUE(configuration_result.is_valid(error_msg)) << error_msg;
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
            kind: "void"
          - name: "P2"
            kind: "void"
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
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is valid
        utils::Formatter error_msg;
        ASSERT_TRUE(configuration_result.is_valid(error_msg)) << error_msg;
    }
}

/**
 * Do not set Yaml version and get default configuration
 * (currently default is v1.0)
 *
 * CASES:
 * - trivial configuration of v1.0
 * - trivial configuration of v2.0 fails
 */
TEST(YamlReaderConfigurationTest, get_ddsrouter_configuration_no_version)
{
    // trivial configuration of v1.0
    {
        const char* yml_configuration =
                R"(
            participant1:
              type: "void"
            participant2:
              type: "void"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is valid
        utils::Formatter error_msg;
        ASSERT_TRUE(configuration_result.is_valid(error_msg)) << error_msg;
    }

    // trivial configuration of v2.0 fails
    {
        const char* yml_configuration =
                R"(
            participants:
              - name: "P1"
                kind: "void"
              - name: "P2"
                kind: void"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is not valid
        utils::Formatter error_msg;
        ASSERT_FALSE(configuration_result.is_valid(error_msg)) << error_msg;
    }
}

/**
 * Test error in version tag
 *
 * CASES:
 * - not existing version
 * - not correct version: specify v1.0 and is v2.0
 * - not correct version: specify v2.0 and is v1.0
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
                kind: "void"
              - name: "P2"
                kind: void"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ASSERT_THROW(
            YamlReaderConfiguration::load_ddsrouter_configuration(yml),
            utils::ConfigurationException);
    }

    // not correct version: specify v1.0 and is v2.0
    {
        // trivial configuration
        const char* yml_configuration =
                R"(
            version: v1.0
            participants:
              - name: "P1"
                kind: "void"
              - name: "P2"
                kind: void"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check is not valid
        utils::Formatter error_msg;
        ASSERT_FALSE(configuration_result.is_valid(error_msg)) << error_msg;
    }

    // not correct version: specify v2.0 and is v1.0
    {
        // trivial configuration
        const char* yml_configuration =
                R"(
            version: v2.0
            participant1:
              type: "void"
            participant2:
              type: "void"
            )";

        Yaml yml = YAML::Load(yml_configuration);

        // Load configuration
        ASSERT_THROW(
            YamlReaderConfiguration::load_ddsrouter_configuration(yml),
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
            kind: "void"
          - name: "P2"
            kind: "void"
        )";
    Yaml yml = YAML::Load(yml_configuration);

    std::vector<unsigned int> test_cases = {1, 2, 10, 20, 42, 100};

    for (unsigned int test_case : test_cases)
    {
        Yaml yml_specs;
        yml_specs[NUMBER_THREADS_TAG] = test_case;
        yml[SPECS_TAG] = yml_specs;

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check threads are correct
        ASSERT_EQ(test_case, configuration_result.advance_options.number_of_threads);
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
            kind: "void"
          - name: "P2"
            kind: "void"
        )";
    Yaml yml = YAML::Load(yml_configuration);

    std::vector<unsigned int> test_cases = {10, 100, 1000, 5000, 10000};

    for (unsigned int test_case : test_cases)
    {
        Yaml yml_specs;
        yml_specs[MAX_HISTORY_DEPTH_TAG] = test_case;
        yml[SPECS_TAG] = yml_specs;

        // Load configuration
        core::configuration::DDSRouterConfiguration configuration_result =
                YamlReaderConfiguration::load_ddsrouter_configuration(yml);

        // Check max history depth is correct
        ASSERT_EQ(test_case, configuration_result.advance_options.max_history_depth);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
