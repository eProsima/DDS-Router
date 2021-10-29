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

#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/exceptions/ConfigurationException.hpp>
#include <databroker/types/RawConfiguration.hpp>
#include <databroker/types/configuration_tags.hpp>

using namespace eprosima::databroker;

/***************
 * CONSTRUCTOR *
 ***************/

/*
 * Add a topic to a list in a yaml
 * If name or type is not given, it will not be added
 */
void add_topic_to_list_to_yaml(
    RawConfiguration& yaml,
    const char* list,
    std::string topic_name = "",
    std::string topic_type = "")
{
    RawConfiguration topic;

    if (topic_name != "")
    {
        topic[TOPIC_NAME_TAG] = topic_name;
    }

    if (topic_type != "")
    {
        topic[TOPIC_TYPE_NAME_TAG] = topic_type;
    }

    yaml[list].push_back(topic);
}


/**
 * Test DatabrokerConfiguration constructor to check it does not fail
 *
 * CASES:
 *  Empty configuration
 *  Random configuration
 */
TEST(DatabrokerConfigurationTest, constructor)
{
    // Empty case
    DatabrokerConfiguration config_empty(RawConfiguration());

    // Random case
    RawConfiguration random_config;
    random_config["RAND_TAG_1"] = "rand_val_1";
    random_config["RAND_TAG_2"] = "rand_val_2";
    random_config["RAND_TAG_3"].push_back(314);
    DatabrokerConfiguration config_random(random_config);
}

/****************************
 * PUBLIC METHODS STD CASES *
 ****************************/

/**
 * Test get participants configurations
 */
TEST(DatabrokerConfigurationTest, participants_configurations)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test get real topics from whitelist
 */
TEST(DatabrokerConfigurationTest, real_topics)
{
    // TODO
    ASSERT_TRUE(false);
}

/*********************************
 * PUBLIC METHODS SPECIFIC CASES *
 *********************************/

/**
 * Test get whitelist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(DatabrokerConfigurationTest, whitelist_wildcard)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test get blacklist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(DatabrokerConfigurationTest, blacklist_wildcard)
{
    // TODO
    ASSERT_TRUE(false);
}

/******************************
 * PUBLIC METHODS ERROR CASES *
 ******************************/

/**
 * Test DatabrokerConfiguration constructor to check it does not fail
 *
 * CASES:
 *  Array as base configuration
 *  Scalar as base configuration
 *  String as base configuration
 */
TEST(DatabrokerConfigurationTest, constructor_fail)
{
    // Array case
    RawConfiguration array_config;
    array_config.push_back("rand_val_1");
    array_config.push_back("rand_val_2");
    EXPECT_THROW(DatabrokerConfiguration dc(array_config), ConfigurationException);

    // Scalar case
    RawConfiguration scalar_config;
    scalar_config = 42;
    EXPECT_THROW(DatabrokerConfiguration dc(scalar_config), ConfigurationException);

    // Scalar case
    RawConfiguration string_config;
    string_config = "non_valid_config";
    EXPECT_THROW(DatabrokerConfiguration dc(string_config), ConfigurationException);
}

/**
 * Test get participants configurations negative cases
 */
TEST(DatabrokerConfigurationTest, participants_configurations_fail)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test get real topics from whitelist negative cases
 */
TEST(DatabrokerConfigurationTest, real_topics_fail)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test get whitelist with wildcards from yaml negative cases
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(DatabrokerConfigurationTest, whitelist_wildcard_fail)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test get blacklist with wildcards from yaml negative cases
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(DatabrokerConfigurationTest, blacklist_wildcard_fail)
{
    // TODO
    ASSERT_TRUE(false);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
