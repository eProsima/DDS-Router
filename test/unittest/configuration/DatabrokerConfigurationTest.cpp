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

#include <databroker/types/configuration_tags.hpp>

using namespace eprosima::databroker;

/***************
 * CONSTRUCTOR *
 ***************/

/**
 * Test DatabrokerConfiguration constructor to check it does not fail
 */
TEST(DatabrokerConfigurationTest, constructor)
{
    // TODO
    ASSERT_TRUE(false);
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
