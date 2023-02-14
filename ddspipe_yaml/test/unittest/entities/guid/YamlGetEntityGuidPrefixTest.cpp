// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

#include <ddspipe_yaml/testing/generate_yaml.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;
using namespace eprosima::ddspipe::core::testing;
using namespace eprosima::ddspipe::yaml::testing;

/**
 * Test read core::types::GuidPrefix from yaml with explicit guid
 *
 * POSITIVE CASES:
 * - std core::types::GuidPrefix
 * - str guidprefix and id
 *
 * NEGATIVE CASES:
 * - empty
 * - incorrect format (not string)
 * - incorrect guid
 */
TEST(YamlGetEntityGuidPrefixTest, get_guidprefix_explicitly)
{
    // std core::types::GuidPrefix
    {
        std::string st = "01.0f.00.00.00.00.00.00.00.00.ff.ff";

        Yaml yml_gp;
        add_field_to_yaml(
            yml_gp,
            YamlField<std::string>(st),
            DISCOVERY_SERVER_GUID_TAG);

        Yaml yml;
        yml["guid_prefix"] = yml_gp;

        core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

        ASSERT_EQ(gp_result, core::types::GuidPrefix(st));
    }

    // str guidprefix and id
    {
        std::string st = "01.0f.00.00.00.00.00.00.00.00.ff.ff";

        Yaml yml_gp;
        add_field_to_yaml(
            yml_gp,
            YamlField<std::string>(st),
            DISCOVERY_SERVER_GUID_TAG);

        // This field must be skipped
        add_field_to_yaml(
            yml_gp,
            YamlField<uint32_t>(0),
            DISCOVERY_SERVER_ID_TAG);

        Yaml yml;
        yml["guid_prefix"] = yml_gp;

        core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

        ASSERT_EQ(gp_result, core::types::GuidPrefix(st));
    }

    // empty
    {
        Yaml yml_gp;
        Yaml yml;
        yml["guid_prefix"] = yml_gp;

        ASSERT_THROW(
            YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST),
            eprosima::utils::ConfigurationException);
    }

    // TODO: this tests requires to modify fastrtps core::types::GuidPrefix >> operator so it returns a non valid guid
    // incorrect format (not string)
    // {
    //     Yaml yml_gp;
    //     add_field_to_yaml(
    //         yml_gp,
    //         YamlField<bool>(false),
    //         DISCOVERY_SERVER_GUID_TAG);

    //     Yaml yml;
    //     yml["guid_prefix"] = yml_gp;

    //     // bool to str in yamlcpp does not cause an error
    //     core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

    //     ASSERT_FALSE(gp_result.is_valid()) << gp_result;
    // }

    // // incorrect guid
    // {
    //     std::string st = "ffff";

    //     Yaml yml_gp;
    //     add_field_to_yaml(
    //         yml_gp,
    //         YamlField<std::string>(st),
    //         DISCOVERY_SERVER_GUID_TAG);

    //     Yaml yml;
    //     yml["guid_prefix"] = yml_gp;

    //     // incorrect guid does not cause an error
    //     core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

    //     ASSERT_FALSE(gp_result.is_valid());
    // }
}

/**
 * Test read core::types::GuidPrefix from yaml given an id
 *
 * POSITIVE CASES:
 * - std ids
 *
 * NEGATIVE CASES:
 * - incorrect format (str instead of int)
 */
TEST(YamlGetEntityGuidPrefixTest, get_guidprefix_id)
{
    // std ids
    {
        std::vector<uint32_t> ids = {0, 2, 12, 234};

        for (uint32_t id : ids)
        {
            Yaml yml_gp;
            add_field_to_yaml(
                yml_gp,
                YamlField<uint32_t>(id),
                DISCOVERY_SERVER_ID_TAG);

            Yaml yml;
            yml["guid_prefix"] = yml_gp;

            core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

            ASSERT_EQ(gp_result, core::types::GuidPrefix(id));
        }
    }

    // incorrect format (str instead of int)
    {
        Yaml yml_gp;

        add_field_to_yaml(
            yml_gp,
            YamlField<std::string>("01.0f.00.00.00.00.00.00.00.00.ff.ff"),
            DISCOVERY_SERVER_ID_TAG);

        Yaml yml;
        yml["guid_prefix"] = yml_gp;

        ASSERT_THROW(YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST),
                eprosima::utils::ConfigurationException);
    }
}

/**
 * Test read core::types::GuidPrefix from yaml with id and using ros
 *
 * POSITIVE CASES:
 * - std ids with ros
 * - std ids without ros
 *
 * NEGATIVE CASES:
 * - incorrect format (str instead of bool)
 */
TEST(YamlGetEntityGuidPrefixTest, get_guidprefix_id_ros)
{
    // std ids with ros
    {
        std::vector<uint32_t> ids = {0, 2, 12, 234};

        for (uint32_t id : ids)
        {
            Yaml yml_gp;

            add_field_to_yaml(
                yml_gp,
                YamlField<uint32_t>(id),
                DISCOVERY_SERVER_ID_TAG);

            add_field_to_yaml(
                yml_gp,
                YamlField<bool>(true),
                DISCOVERY_SERVER_ID_ROS_TAG);

            Yaml yml;
            yml["guid_prefix"] = yml_gp;

            core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

            ASSERT_EQ(gp_result, core::types::GuidPrefix(true, id));
        }
    }

    // std ids without ros
    {
        std::vector<uint32_t> ids = {0, 2, 12, 234};

        for (uint32_t id : ids)
        {
            Yaml yml_gp;

            add_field_to_yaml(
                yml_gp,
                YamlField<uint32_t>(id),
                DISCOVERY_SERVER_ID_TAG);

            add_field_to_yaml(
                yml_gp,
                YamlField<bool>(false),
                DISCOVERY_SERVER_ID_ROS_TAG);

            Yaml yml;
            yml["guid_prefix"] = yml_gp;

            core::types::GuidPrefix gp_result = YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST);

            ASSERT_EQ(gp_result, core::types::GuidPrefix(false, id));
        }
    }

    // incorrect format (str instead of int)
    {
        Yaml yml_gp;

        add_field_to_yaml(
            yml_gp,
            YamlField<uint32_t>(0),
            DISCOVERY_SERVER_ID_TAG);

        add_field_to_yaml(
            yml_gp,
            YamlField<std::string>("01.0f.00.00.00.00.00.00.00.00.ff.ff"),
            DISCOVERY_SERVER_ID_ROS_TAG);

        Yaml yml;
        yml["guid_prefix"] = yml_gp;

        ASSERT_THROW(YamlReader::get<core::types::GuidPrefix>(yml, "guid_prefix", LATEST),
                eprosima::utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
