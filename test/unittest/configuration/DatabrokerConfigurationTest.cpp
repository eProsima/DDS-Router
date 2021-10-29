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
#include <databroker/types/configuration_tags.hpp>
#include <databroker/types/RawConfiguration.hpp>
#include <databroker/types/topic/WildcardTopic.hpp>

using namespace eprosima::databroker;

/***************
 * CONSTRUCTOR *
 ***************/

/*
 * Add a topic to a list in a yaml
 * If name or type is not given, that tag will not be added
 */
void add_topic_to_list_to_yaml(
    RawConfiguration& yaml,
    const char* list_tag,
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

    yaml[list_tag].push_back(topic);
}

/*
 * Add a list of topics to a list in a yaml
 * If name or type is not given, that tag will not be added
 */
void add_topics_to_list_to_yaml(
    RawConfiguration& yaml,
    const char* list_tag,
    std::set<std::pair<std::string, std::string>> names)
{
    for (std::pair<std::string, std::string> name : names)
    {
        add_topic_to_list_to_yaml(yaml, list_tag, name.first, name.second);
    }
}

/*
 * Check if a topic is inside a list returned by whitelist or blacklist Databroker methods
 */
bool topic_in_list(
    std::list<AbstractTopic*> list,
    WildcardTopic compared_topic)
{
    for (AbstractTopic* topic : list)
    {
        // Check class and internal variables
        if (typeid(*topic) == typeid(compared_topic) &&
            compared_topic == *topic)
        {
            return true;
        }
    }
    return false;
}

/*
 * Check if a topic is inside a list returned by real_topics Databroker methods
 */
bool topic_in_real_list(
    std::set<RealTopic> list,
    RealTopic compared_topic)
{
    for (RealTopic topic : list)
    {
        // Check class and internal variables
        if (typeid(topic) == typeid(compared_topic) &&
            compared_topic == topic)
        {
            return true;
        }
    }
    return false;
}

/*
 * Random Real topic names to test different configurations
 */
std::set<std::pair<std::string, std::string>> random_real_topic_names()
{
    return
        {
            {"TopicName1", "TopicType1"},
            {"TopicName2", "TopicType2"},
            {"TopicName3", "TopicType3"},

            {"rt/chatter", "std::str::main::other_namespace"},

            {"real/rare_topic.name", "real/rare_topic.type"},
        };
}

/*
 * Random Non Real topic names to test different configurations
 */
std::set<std::pair<std::string, std::string>> random_abstract_topic_names()
{
    return
        {
            {"rt/chatter", "*"},
            {"*", "std::str"},

            {"rt/chatter", "std::str*"},
            {"rt/chatter/*", "std::str"},
            {"rt/chatter/*", "std::str*"},

            {"rt/chatter", "*::std::str"},
            {"*/rt/chatter", "std::str"},
            {"*/rt/chatter", "*::std::str"},

            {"rt/chatter", "*::std::str*"},
            {"*/rt/chatter/*", "std::str"},
            {"*/rt/chatter/*", "*::std::str*"},

            {"rt/chatter", ""},
            {"*/rt/chatter/*", ""},
        };
}

/*
 * Unioin of real and non real topic names
 */
std::set<std::pair<std::string, std::string>> random_topic_names()
{
    std::set<std::pair<std::string, std::string>> all_topics;
    std::set<std::pair<std::string, std::string>> real_topics = random_real_topic_names();
    std::set<std::pair<std::string, std::string>> abs_topics = random_abstract_topic_names();
    std::set_union(
        real_topics.begin(), real_topics.end(),
        abs_topics.begin(), abs_topics.end(),
        std::inserter(all_topics, all_topics.begin()));

    return all_topics;
}

/*
 * Random participant id
 *
 * TODO: create really random names
 */
std::string random_participant_name(uint16_t seed)
{
    return std::string("PartName_") + std::to_string(seed);
}

/*
 * Random participant configuration
 *
 * TODO: create really random configurations
 */
RawConfiguration random_participant_configuration(uint16_t seed)
{
    RawConfiguration config;

    for (int i=(seed); i>0; i--)
    {
        std::string tag("tag" + std::to_string(i));

        if (i % 4 == 0)
        {
            // Each 4 tags add an array
            config[tag].push_back("value1");
            config[tag].push_back("value2");
        }
        else if(i % 2 == 0)
        {
            // Each 4 tags add a map
            RawConfiguration sub_map;
            sub_map["x"] = "y";
            sub_map["a"] = "b";
            config[tag] = sub_map;
        }
        else
        {
            // The rest are direct values
            config[tag] = "Random value";
        }
    }

    return config;
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
 *
 * CASES:
 *  Empty configuration
 *  Other tags that are not participant valid ids
 *  One Participant Configuration
 *  Many Participant Configurations
 */
TEST(DatabrokerConfigurationTest, participants_configurations)
{
    // Empty configuration
    RawConfiguration yaml1;
    DatabrokerConfiguration config1(yaml1);
    EXPECT_TRUE(config1.participants_configurations().empty());

    // Other tags that are not participant valid ids
    RawConfiguration yaml2;
    add_topics_to_list_to_yaml(yaml2, WHITELIST_TAG, random_abstract_topic_names());
    add_topics_to_list_to_yaml(yaml2, BLACKLIST_TAG, random_real_topic_names());
    DatabrokerConfiguration config2(yaml2);
    EXPECT_TRUE(config2.participants_configurations().empty());

    // One Participant Configuration
    RawConfiguration listening_addresses;
    RawConfiguration address1;
    RawConfiguration address2;
    address1["ip"] = "127.0.0.1";
    address1["port"] = "31415";
    listening_addresses.push_back(address1);
    address2["ip"] = "8.8.8.8";
    address2["port"] = "6666";
    listening_addresses.push_back(address2);

    RawConfiguration participant_config;
    participant_config["type"] = "wan";
    participant_config["listening-addresses"] = listening_addresses;

    std::string participant_name_str = "wanParticipant";
    ParticipantId participant_name(participant_name_str);
    RawConfiguration yaml3;
    yaml3[participant_name_str] = participant_config;

    DatabrokerConfiguration config3(yaml3);
    auto result3 = config3.participants_configurations();
    ASSERT_EQ(1, result3.size());
    EXPECT_TRUE(result3.find(participant_name) != result3.end());
    EXPECT_EQ(participant_config, result3[participant_name]);

    // Many Participant Configurations
    RawConfiguration yaml4;
    for (int i=0; i<10; i++)
    {
        yaml4[random_participant_name(i)] = random_participant_configuration(i);
    }
    DatabrokerConfiguration config4(yaml4);
    auto result4 = config4.participants_configurations();

    for (int i=0; i<10; i++)
    {
        ParticipantId id(random_participant_name(i));
        // WARNING: comparing two YAMLs is not correclty done in some occasions
        // TODO: check that are actually the same yaml
        ASSERT_EQ(result4[id].size(), random_participant_configuration(i).size());
        // std::cout << result4[id].size() << std::endl; // -> this goes from 0 to 9
    }
}

/**
 * Test get real topics from whitelist
 *
 * CASES:
 *  Empty configuration
 *  Empty whitelist
 *  Whitelist with only non Real topics
 *  Whitelist with only Real topics
 *  Whitelist with random topics
 */
TEST(DatabrokerConfigurationTest, real_topics)
{
    // Empty configuration
    RawConfiguration yaml1;
    DatabrokerConfiguration config1(yaml1);
    EXPECT_TRUE(config1.real_topics().empty());

    // Empty whitelist
    RawConfiguration yaml2;
    add_topic_to_list_to_yaml(yaml2, BLACKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, BLACKLIST_TAG, "topic2", "type2");
    DatabrokerConfiguration config2(yaml2);
    EXPECT_TRUE(config2.real_topics().empty());

    // Whitelist with only non Real topics
    RawConfiguration yaml3;
    add_topics_to_list_to_yaml(yaml3, WHITELIST_TAG, random_abstract_topic_names());
    DatabrokerConfiguration config3(yaml3);
    EXPECT_TRUE(config3.real_topics().empty());

    // Whitelist with only non Real topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, WHITELIST_TAG, random_real_topic_names());
    DatabrokerConfiguration config4(yaml4);
    auto result4 = config4.real_topics();
    EXPECT_FALSE(result4.empty());
    for (auto topic_name : random_real_topic_names())
    {
        RealTopic topic(topic_name.first, topic_name.second);
        EXPECT_TRUE(topic_in_real_list(result4, topic));
    }

    // Whitelist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, WHITELIST_TAG, random_topic_names());
    DatabrokerConfiguration config5(yaml5);
    auto result5 = config5.real_topics();
    EXPECT_FALSE(result5.empty());

    uint16_t real_topics = 0;
    for (auto topic_name : random_topic_names())
    {
        bool real_topic = RealTopic::is_real_topic(topic_name.first, topic_name.second);

        if (real_topic)
        {
            ++real_topics;
            RealTopic topic(topic_name.first, topic_name.second);
            EXPECT_TRUE(topic_in_real_list(result5, topic));
        }
    }
    EXPECT_EQ(real_topics, result5.size());
}

/*********************************
 * PUBLIC METHODS SPECIFIC CASES *
 *********************************/

/**
 * Test get whitelist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Empty configuration
 *  Empty whitelist
 *  Whitelist with some examples
 *  Whitelist with random topics
 *  Whitelist and blacklist with random topics
 */
TEST(DatabrokerConfigurationTest, whitelist_wildcard)
{
    // Empty configuration
    RawConfiguration yaml1;
    DatabrokerConfiguration config1(yaml1);
    EXPECT_TRUE(config1.whitelist().empty());

    // Empty whitelist
    RawConfiguration yaml2;
    add_topic_to_list_to_yaml(yaml2, BLACKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, BLACKLIST_TAG, "topic2", "type2");
    DatabrokerConfiguration config2(yaml2);
    EXPECT_TRUE(config2.whitelist().empty());

    // Empty whitelist
    RawConfiguration yaml3;
    add_topic_to_list_to_yaml(yaml3, WHITELIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml3, WHITELIST_TAG, "topic2*", "type2*");
    DatabrokerConfiguration config3(yaml3);
    auto result3 = config3.whitelist();
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic("topic1", "type1")));
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic("topic2*", "type2*")));

    // Whitelist with random topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, WHITELIST_TAG, random_topic_names());
    DatabrokerConfiguration config4(yaml4);
    EXPECT_EQ(config4.whitelist().size(), random_topic_names().size());

    // Whitelist and blacklist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, WHITELIST_TAG, random_abstract_topic_names());
    add_topics_to_list_to_yaml(yaml5, BLACKLIST_TAG, random_real_topic_names());
    DatabrokerConfiguration config5(yaml5);
    EXPECT_EQ(config5.whitelist().size(), random_abstract_topic_names().size());
}

/**
 * Test get blacklist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Empty configuration
 *  Empty blacklist
 *  Blacklist with some examples
 *  Blacklist with random topics
 *  Blacklist and whitelist with random topics
 */
TEST(DatabrokerConfigurationTest, blacklist_wildcard)
{
    // Empty configuration
    RawConfiguration yaml1;
    DatabrokerConfiguration config1(yaml1);
    EXPECT_TRUE(config1.blacklist().empty());

    // Empty blacklist
    RawConfiguration yaml2;
    add_topic_to_list_to_yaml(yaml2, WHITELIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, WHITELIST_TAG, "topic2", "type2");
    DatabrokerConfiguration config2(yaml2);
    EXPECT_TRUE(config2.blacklist().empty());

    // Empty blacklist
    RawConfiguration yaml3;
    add_topic_to_list_to_yaml(yaml3, BLACKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml3, BLACKLIST_TAG, "topic2*", "type2*");
    DatabrokerConfiguration config3(yaml3);
    auto result3 = config3.blacklist();
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic("topic1", "type1")));
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic("topic2*", "type2*")));

    // Blacklist with random topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, BLACKLIST_TAG, random_topic_names());
    DatabrokerConfiguration config4(yaml4);
    EXPECT_EQ(config4.blacklist().size(), random_topic_names().size());

    // Blacklist and whitelist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, BLACKLIST_TAG, random_abstract_topic_names());
    add_topics_to_list_to_yaml(yaml5, WHITELIST_TAG, random_real_topic_names());
    DatabrokerConfiguration config5(yaml5);
    EXPECT_EQ(config5.blacklist().size(), random_abstract_topic_names().size());
}

/**
 * Test get blacklist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(DatabrokerConfigurationTest, whitelist_and_blacklist)
{
    RawConfiguration yaml;
    add_topics_to_list_to_yaml(yaml, WHITELIST_TAG, random_real_topic_names());
    add_topics_to_list_to_yaml(yaml, BLACKLIST_TAG, random_abstract_topic_names());
    DatabrokerConfiguration config(yaml);
    EXPECT_EQ(config.whitelist().size(), random_real_topic_names().size());
    EXPECT_EQ(config.blacklist().size(), random_abstract_topic_names().size());
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

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
