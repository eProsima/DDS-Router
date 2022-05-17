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

#include <algorithm>
#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_utils/exception/utils::ConfigurationException.hpp>
#include <ddsrouter_core/types/configuration_tags.hpp>
#include <ddsrouter_core/types/RawConfiguration.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

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
 * Add a tag with empty value
 */
void add_empty_tag_to_yaml(
        RawConfiguration& yaml,
        std::string tag)
{
    yaml[tag] = RawConfiguration();
}

/*
 * Check if a topic is inside a list returned by allowlist or blocklist DDSRouter methods
 */
bool topic_in_list(
        std::list<std::shared_ptr<FilterTopic>> list,
        WildcardTopic compared_topic)
{
    for (std::shared_ptr<FilterTopic> topic : list)
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
 * Check if a topic is inside a list returned by real_topics DDSRouter methods
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
 * Random Non Valid topics
 */
std::set<std::pair<std::string, std::string>> random_non_valid_topic_names()
{
    return
        {
            {"", ""},
            {"", "*"},
        };
}

/*
 * Random Non Real topic names to test different configurations
 */
std::set<std::pair<std::string, std::string>> random_filter_topic_names()
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
    std::set<std::pair<std::string, std::string>> abs_topics = random_filter_topic_names();
    std::set_union(
        real_topics.begin(), real_topics.end(),
        abs_topics.begin(), abs_topics.end(),
        std::inserter(all_topics, all_topics.begin()));

    std::set<std::pair<std::string, std::string>> non_valid_topics = random_non_valid_topic_names();
    all_topics.insert(non_valid_topics.begin(), non_valid_topics.end());

    return all_topics;
}

/*
 * Number of valid topics in random_topic_names()
 */
size_t random_topic_names_number_valid_topics()
{
    return random_real_topic_names().size() + random_filter_topic_names().size();
}

/*
 * Random participant id
 *
 * TODO: create really random names
 */
std::string random_participant_name(
        uint16_t seed)
{
    return std::string("PartName_") + std::to_string(seed);
}

/*
 * Random participant kind
 *
 * WARNING: the max_types_available must be updated with each new type added
 */
ParticipantKind random_participant_kind(
        uint16_t seed = 0)
{
    // Avoid Invalid type
    return AllValidParticipantKinds[seed % AllValidParticipantKinds.size()];
}

/*
 * Random participant configuration
 *
 * TODO: create really random configurations
 */
RawConfiguration random_participant_configuration(
        uint16_t seed)
{
    RawConfiguration config;

    for (int i = (seed); i > 0; i--)
    {
        std::string tag("tag" + std::to_string(i));

        if (i % 4 == 0)
        {
            // Each 4 tags add an array
            config[tag].push_back("value1");
            config[tag].push_back("value2");
        }
        else if (i % 2 == 0)
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

    config[PARTICIPANT_KIND_TAG] = random_participant_kind().to_string();

    return config;
}

/**
 * Test Configuration constructor to check it does not fail
 *
 * CASES:
 *  Empty configuration
 *  Random configuration
 */
TEST(ConfigurationTest, constructor)
{
    // Empty case
    RawConfiguration empty_yaml;
    DDSRouterConfiguration config_empty(empty_yaml);

    // Random case
    RawConfiguration random_config;
    random_config["RAND_TAG_1"] = "rand_val_1";
    random_config["RAND_TAG_2"] = "rand_val_2";
    random_config["RAND_TAG_3"].push_back(314);
    DDSRouterConfiguration config_random(random_config);
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
 *
 * TODO: Change yaml tags for their proper external tags (constexpr)
 */
TEST(ConfigurationTest, participants_configurations)
{
    {
        // Empty configuration
        RawConfiguration yaml1;
        DDSRouterConfiguration config1(yaml1);
        EXPECT_TRUE(config1.participants_configurations().empty());
    }

    {
        // Other tags that are not participant valid ids
        RawConfiguration yaml2;
        add_topics_to_list_to_yaml(yaml2, ALLOWLIST_TAG, random_filter_topic_names());
        add_topics_to_list_to_yaml(yaml2, BLOCKLIST_TAG, random_real_topic_names());
        DDSRouterConfiguration config2(yaml2);
        EXPECT_TRUE(config2.participants_configurations().empty());
    }

    {
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
        participant_config[PARTICIPANT_KIND_TAG] = random_participant_kind().to_string();
        participant_config["listening-addresses"] = listening_addresses;

        std::string participant_name_str = "wanParticipant";
        ParticipantId participant_name(participant_name_str);
        RawConfiguration yaml3;
        yaml3[participant_name_str] = participant_config;

        DDSRouterConfiguration config3(yaml3);
        auto result3 = config3.participants_configurations();
        ASSERT_EQ(1, result3.size());
        EXPECT_EQ(participant_name, result3.front().id());
        EXPECT_EQ(ParticipantConfiguration(ParticipantId(participant_name_str), participant_config), result3.front());
    }

    {
        // Many Participant Configurations
        uint16_t participants_num = 10;
        RawConfiguration yaml4;
        for (int i = 0; i < participants_num; i++)
        {
            yaml4[random_participant_name(i)] = random_participant_configuration(i);
        }
        DDSRouterConfiguration config4(yaml4);
        auto result4 = config4.participants_configurations();
        ASSERT_EQ(result4.size(), participants_num);

        // For every participant in participants_configurations, check that the id is inside the actual ids stored
        // TODO: check the configurations are actually the same, implement yaml compare
        for (int i = 0; i < participants_num; i++)
        {
            ParticipantId expected_id(random_participant_name(i));
            bool in_configurations = false;
            for (auto part_config: result4)
            {
                // They may not be sorted, so it must be checked that this is the actual participant config
                // it is being tested
                if (part_config.id() == expected_id)
                {
                    in_configurations  = true;
                    break;
                }
            }
            ASSERT_TRUE(in_configurations);
        }
    }
}

/**
 * Test get real topics from allowlist
 *
 * CASES:
 *  Empty configuration
 *  Empty allowlist
 *  Allowlist with only non Real topics
 *  Allowlist with only Real topics
 *  Allowlist with random topics
 */
TEST(ConfigurationTest, real_topics)
{
    // Empty configuration
    RawConfiguration yaml1;
    DDSRouterConfiguration config1(yaml1);
    EXPECT_TRUE(config1.real_topics().empty());

    // Empty allowlist
    RawConfiguration yaml2;
    add_topic_to_list_to_yaml(yaml2, BLOCKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, BLOCKLIST_TAG, "topic2", "type2");
    DDSRouterConfiguration config2(yaml2);
    EXPECT_TRUE(config2.real_topics().empty());

    // Allowlist with only non Real topics
    RawConfiguration yaml3;
    add_topics_to_list_to_yaml(yaml3, ALLOWLIST_TAG, random_filter_topic_names());
    DDSRouterConfiguration config3(yaml3);
    EXPECT_TRUE(config3.real_topics().empty());

    // Allowlist with only Real topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, ALLOWLIST_TAG, random_real_topic_names());
    DDSRouterConfiguration config4(yaml4);
    auto result4 = config4.real_topics();
    EXPECT_FALSE(result4.empty());
    for (auto random_topic : random_real_topic_names())
    {
        RealTopic topic(random_topic.first, random_topic.second);
        EXPECT_TRUE(topic_in_real_list(result4, topic));
    }

    // Allowlist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, ALLOWLIST_TAG, random_topic_names());
    DDSRouterConfiguration config5(yaml5);
    auto result5 = config5.real_topics();
    EXPECT_FALSE(result5.empty());

    uint16_t real_topics = 0;
    for (auto random_topic : random_topic_names())
    {
        bool real_topic = RealTopic::is_real_topic(random_topic.first, random_topic.second);

        if (real_topic)
        {
            ++real_topics;
            RealTopic topic(random_topic.first, random_topic.second);
            EXPECT_TRUE(topic_in_real_list(result5, topic)) << topic;
        }
    }
    EXPECT_EQ(real_topics, result5.size());
}

/*********************************
* PUBLIC METHODS SPECIFIC CASES *
*********************************/

/**
 * Test get allowlist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Empty configuration
 *  Empty allowlist
 *  Allowlist with some examples
 *  Allowlist with random topics
 *  Allowlist and blocklist with random topics
 */
TEST(ConfigurationTest, allowlist_wildcard)
{
    // Empty configuration
    RawConfiguration yaml1;
    DDSRouterConfiguration config1(yaml1);
    EXPECT_TRUE(config1.allowlist().empty());

    // Empty allowlist
    RawConfiguration yaml2;
    add_empty_tag_to_yaml(yaml2, ALLOWLIST_TAG);
    add_topic_to_list_to_yaml(yaml2, BLOCKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, BLOCKLIST_TAG, "topic2", "type2");
    DDSRouterConfiguration config2(yaml2);
    EXPECT_TRUE(config2.allowlist().empty());

    // Empty allowlist
    RawConfiguration yaml3;
    add_topic_to_list_to_yaml(yaml3, ALLOWLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml3, ALLOWLIST_TAG, "topic2*", "type2*");
    DDSRouterConfiguration config3(yaml3);
    auto result3 = config3.allowlist();
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic(std::string("topic1"), std::string("type1"))));
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic(std::string("topic2*"), std::string("type2*"))));

    // Allowlist with random topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, ALLOWLIST_TAG, random_topic_names());
    DDSRouterConfiguration config4(yaml4);
    EXPECT_EQ(config4.allowlist().size(), random_topic_names_number_valid_topics());

    // Allowlist and blocklist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, ALLOWLIST_TAG, random_filter_topic_names());
    add_topics_to_list_to_yaml(yaml5, BLOCKLIST_TAG, random_real_topic_names());
    DDSRouterConfiguration config5(yaml5);
    EXPECT_EQ(config5.allowlist().size(), random_filter_topic_names().size());
}

/**
 * Test get blocklist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Empty configuration
 *  Empty blocklist
 *  Blocklist with some examples
 *  Blocklist with random topics
 *  Blocklist and allowlist with random topics
 */
TEST(ConfigurationTest, blocklist_wildcard)
{
    // Empty configuration
    RawConfiguration yaml1;
    DDSRouterConfiguration config1(yaml1);
    EXPECT_TRUE(config1.blocklist().empty());

    // Empty blocklist
    RawConfiguration yaml2;
    add_empty_tag_to_yaml(yaml2, BLOCKLIST_TAG);
    add_topic_to_list_to_yaml(yaml2, ALLOWLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml2, ALLOWLIST_TAG, "topic2", "type2");
    DDSRouterConfiguration config2(yaml2);
    EXPECT_TRUE(config2.blocklist().empty());

    // Empty blocklist
    RawConfiguration yaml3;
    add_topic_to_list_to_yaml(yaml3, BLOCKLIST_TAG, "topic1", "type1");
    add_topic_to_list_to_yaml(yaml3, BLOCKLIST_TAG, "topic2*", "type2*");
    DDSRouterConfiguration config3(yaml3);
    auto result3 = config3.blocklist();
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic(std::string("topic1"), std::string("type1"))));
    EXPECT_TRUE(topic_in_list(result3, WildcardTopic(std::string("topic2*"), std::string("type2*"))));

    // Blocklist with random topics
    RawConfiguration yaml4;
    add_topics_to_list_to_yaml(yaml4, BLOCKLIST_TAG, random_topic_names());
    DDSRouterConfiguration config4(yaml4);
    EXPECT_EQ(config4.blocklist().size(), random_topic_names_number_valid_topics());

    // Blocklist and allowlist with random topics
    RawConfiguration yaml5;
    add_topics_to_list_to_yaml(yaml5, BLOCKLIST_TAG, random_filter_topic_names());
    add_topics_to_list_to_yaml(yaml5, ALLOWLIST_TAG, random_real_topic_names());
    DDSRouterConfiguration config5(yaml5);
    EXPECT_EQ(config5.blocklist().size(), random_filter_topic_names().size());
}

/**
 * Test get blocklist with wildcards from yaml
 *
 * TODO: when regex is implemented, create a common test case
 */
TEST(ConfigurationTest, allowlist_and_blocklist)
{
    RawConfiguration yaml;
    add_topics_to_list_to_yaml(yaml, ALLOWLIST_TAG, random_real_topic_names());
    add_topics_to_list_to_yaml(yaml, BLOCKLIST_TAG, random_filter_topic_names());
    DDSRouterConfiguration config(yaml);
    EXPECT_EQ(config.allowlist().size(), random_real_topic_names().size());
    EXPECT_EQ(config.blocklist().size(), random_filter_topic_names().size());
}

/******************************
* PUBLIC METHODS ERROR CASES *
******************************/

/**
 * Test DDSRouterConfiguration constructor to check it does not fail
 *
 * CASES:
 *  Array as base configuration
 *  Scalar as base configuration
 *  String as base configuration
 */
TEST(ConfigurationTest, constructor_fail)
{
    // Array case
    RawConfiguration array_config;
    array_config.push_back("rand_val_1");
    array_config.push_back("rand_val_2");
    EXPECT_THROW(DDSRouterConfiguration dc(array_config), utils::ConfigurationException);

    // Scalar case
    RawConfiguration scalar_config;
    scalar_config = 42;
    EXPECT_THROW(DDSRouterConfiguration dc(scalar_config), utils::ConfigurationException);

    // Scalar case
    RawConfiguration string_config;
    string_config = "non_valid_config";
    EXPECT_THROW(DDSRouterConfiguration dc(string_config), utils::ConfigurationException);
}

/**
 * Test get participants configurations negative cases
 */
TEST(ConfigurationTest, participants_configurations_fail)
{
    // There is currently no way to induce an error when getting participant configurations
    ASSERT_TRUE(true);
}

/**
 * Test get real topics from allowlist negative cases
 *
 * CASES:
 *  Map instead of array in topics
 */
TEST(ConfigurationTest, real_topics_fail)
{
    // Map instead of array in allowlist
    RawConfiguration map_config;
    map_config["key1"] = "value1";
    RawConfiguration yaml1;
    yaml1[ALLOWLIST_TAG] = map_config;
    DDSRouterConfiguration dc(yaml1);
    EXPECT_THROW(dc.real_topics(), utils::ConfigurationException);
}

/**
 * Test get allowlist with wildcards from yaml negative cases
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Map instead of array in topics
 */
TEST(ConfigurationTest, allowlist_wildcard_fail)
{
    // Map instead of array in allowlist
    RawConfiguration map_config;
    map_config["key1"] = "value1";
    RawConfiguration yaml1;
    yaml1[ALLOWLIST_TAG] = map_config;
    DDSRouterConfiguration dc(yaml1);
    EXPECT_THROW(dc.allowlist(), utils::ConfigurationException);
}

/**
 * Test get blocklist with wildcards from yaml negative cases
 *
 * TODO: when regex is implemented, create a common test case
 *
 * CASES:
 *  Map instead of array in topics
 */
TEST(ConfigurationTest, blocklist_wildcard_fail)
{
    // Map instead of array in blocklist
    RawConfiguration map_config;
    map_config["key1"] = "value1";
    RawConfiguration yaml1;
    yaml1[BLOCKLIST_TAG] = map_config;
    DDSRouterConfiguration dc(yaml1);
    EXPECT_THROW(dc.blocklist(), utils::ConfigurationException);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
