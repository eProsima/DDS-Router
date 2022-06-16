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

#include <ddsrouter_core/configuration/payload_pool/PoolConfig.h>
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

        // Is valid if it does not throw
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

        // Is valid if it does not throw
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

        // Is valid if it does not throw
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
        ASSERT_THROW(
            core::configuration::DDSRouterConfiguration configuration_result =
            YamlReaderConfiguration::load_ddsrouter_configuration(yml)
            , utils::ConfigurationException);
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
        ASSERT_THROW(
            core::configuration::DDSRouterConfiguration configuration_result =
            YamlReaderConfiguration::load_ddsrouter_configuration(yml),
            utils::ConfigurationException);
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

core::configuration::DDSRouterConfiguration test_threads_aux(
        std::string threads_str)
{
    std::string yaml_str = std::string(
        R"(
        version: v2.0
        participants:
          - name: "P1"
            kind: "void"
          - name: "P2"
            kind: "void"
        internal:
            THREADS_KEY_VAL
        )");

    const std::string key_str = "THREADS_KEY_VAL";

    yaml_str.replace(yaml_str.find(key_str), key_str.length(), threads_str);

    Yaml yml = YAML::Load(yaml_str.c_str());

    return YamlReaderConfiguration::load_ddsrouter_configuration(yml);
}

/**
 * Test threads count
 *
 * CASES:
 * - 1 thread
 * - 27 thread
 * - 0 threads throws
 * - threads exceeding max throws
 * - negative threads throws
 * - empty threads string throws
 * - not a number throws
 * - threads not set implies default
 */
TEST(YamlReaderConfigurationTest, threads_count)
{
    // one
    {
        auto configuration = test_threads_aux("threads: 1");
        ASSERT_EQ(configuration.threads(), 1);
    }
    // specific non-one number
    {
        auto configuration = test_threads_aux("threads: 27");
        ASSERT_EQ(configuration.threads(), 27);
    }
    // zero
    {
        ASSERT_THROW(
            auto configuration = test_threads_aux("threads: 0"),
            utils::ConfigurationException);
    }
    // greater than maximum
    {
        ASSERT_THROW(
            auto configuration = test_threads_aux("threads: " + std::to_string(core::configuration::MAX_THREADS + 1)),
            utils::ConfigurationException);
    }
    // negative:
    {
        ASSERT_THROW(
            auto configuration = test_threads_aux("threads: -1"),
            utils::ConfigurationException);
    }
    // empty:
    {
        ASSERT_THROW(
            auto configuration = test_threads_aux("threads: "),
            utils::ConfigurationException);
    }
    // not a number
    {
        ASSERT_THROW(
            auto configuration = test_threads_aux("threads: nonumber"),
            utils::ConfigurationException);
    }
    // unspecified is default
    {
        auto configuration = test_threads_aux("");
        ASSERT_EQ(configuration.threads(), core::configuration::DEFAULT_THREADS);
        ASSERT_EQ(configuration.payload_pool_granularity(), eprosima::fastrtps::rtps::recycle::DEFAULT_GRANULARITY);
        ASSERT_EQ(
            configuration.payload_pool_configuration().payload_initial_size,
            eprosima::fastrtps::rtps::recycle::DEFAULT_PAYLOAD_SIZE);
        ASSERT_EQ(
            configuration.payload_pool_configuration().initial_size,
            eprosima::fastrtps::rtps::recycle::DEFAULT_MIN_ELEMENTS);
        ASSERT_EQ(
            configuration.payload_pool_configuration().maximum_size,
            eprosima::fastrtps::rtps::recycle::DEFAULT_MAX_ELEMENTS);
    }
}

core::configuration::DDSRouterConfiguration test_payload_pool_config_aux(
        unsigned int pp_granularity,
        unsigned int prealloc_payload_size,
        unsigned int prealloc_min_elements,
        unsigned int prealloc_max_elements)
{
    std::string yaml_str = std::string(
        R"(
        version: v2.0
        participants:
          - name: "P1"
            kind: "void"
          - name: "P2"
            kind: "void"
        internal:
            payload_pool_granularity: PAYLOAD_POOL_GRANULARITY
            prealloc_payload_size: PREALLOC_PAYLOAD_SIZE
            prealloc_min_elements: PREALLOC_MIN_ELEMENTS
            prealloc_max_elements: PREALLOC_MAX_ELEMENTS
        )");

    const std::string granularity_key_str = "PAYLOAD_POOL_GRANULARITY";
    const std::string prealloc_payload_size_key_str = "PREALLOC_PAYLOAD_SIZE";
    const std::string prealloc_min_elements_key_str = "PREALLOC_MIN_ELEMENTS";
    const std::string prealloc_max_elements_key_str = "PREALLOC_MAX_ELEMENTS";

    yaml_str.replace(yaml_str.find(granularity_key_str), granularity_key_str.length(), std::to_string(pp_granularity));
    yaml_str.replace(yaml_str.find(prealloc_payload_size_key_str),
            prealloc_payload_size_key_str.length(), std::to_string(prealloc_payload_size));
    yaml_str.replace(yaml_str.find(prealloc_min_elements_key_str),
            prealloc_min_elements_key_str.length(), std::to_string(prealloc_min_elements));
    yaml_str.replace(yaml_str.find(prealloc_max_elements_key_str),
            prealloc_max_elements_key_str.length(), std::to_string(prealloc_max_elements));

    Yaml yml = YAML::Load(yaml_str.c_str());

    return YamlReaderConfiguration::load_ddsrouter_configuration(yml);
}

/**
 * Test payload pool configuration parameters
 *
 * CASES:
 * - 0 values
 * - non-zero values
 */
TEST(YamlReaderConfigurationTest, payload_pool_configuration)
{
    {
        auto configuration = test_payload_pool_config_aux(0, 0, 0, 0);
        ASSERT_EQ(configuration.payload_pool_granularity(), 0);
        ASSERT_EQ(configuration.payload_pool_configuration().payload_initial_size, 0);
        ASSERT_EQ(configuration.payload_pool_configuration().initial_size, 0);
        ASSERT_EQ(configuration.payload_pool_configuration().maximum_size, 0);
    }
    {
        auto configuration = test_payload_pool_config_aux(13, 12, 27, 44);
        ASSERT_EQ(configuration.payload_pool_granularity(), 13);
        ASSERT_EQ(configuration.payload_pool_configuration().payload_initial_size, 12);
        ASSERT_EQ(configuration.payload_pool_configuration().initial_size, 27);
        ASSERT_EQ(configuration.payload_pool_configuration().maximum_size, 44);
    }

}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
