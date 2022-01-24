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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

#include "../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

void real_topic_to_yaml(
    Yaml& yml,
    const test::YamlField<std::string>& name,
    const test::YamlField<std::string>& type,
    const test::YamlField<bool>& keyed)
{
    test::add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    test::add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    test::add_field_to_yaml(yml, keyed, TOPIC_KIND_TAG);
}

void compare_topic(
    RealTopic topic,
    std::string name,
    std::string type,
    bool keyed)
{
    ASSERT_EQ(topic.topic_name(), name);
    ASSERT_EQ(topic.topic_type(), type);
    ASSERT_EQ(topic.topic_with_key(), keyed);
}

/**
 * Test read RealTopic from yaml
 *
 * POSITIVE CASES:
 * - Topic Std
 * - Topic with key
 * - Topic with no key
 *
 * NEGATIVE CASES:
 * - Empty
 * - Topic without name
 * - Topic without type
 */
TEST(YamlGetEntityTest, get_real_topic)
{
    std::string name = "topic_name";
    std::string type = "topic_type";

    // Topic Std
    {
        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        RealTopic topic = YamlReader::get<RealTopic>(yml, "topic");

        compare_topic(topic, name, type, false); // By default no keyed
    }

    // Topic with key
    {
        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(true));

        Yaml yml;
        yml["topic"] = yml_topic;

        RealTopic topic = YamlReader::get<RealTopic>(yml, "topic");

        compare_topic(topic, name, type, true);
    }

    // Topic with no key
    {
        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false));

        Yaml yml;
        yml["topic"] = yml_topic;

        RealTopic topic = YamlReader::get<RealTopic>(yml, "topic");

        compare_topic(topic, name, type, false);
    }

    // Topic without name
    {
        Yaml yml;

        ASSERT_THROW(YamlReader::get<RealTopic>(yml, "topic"), ConfigurationException);
    }

    // Topic without name
    {
        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(),
            test::YamlField<std::string>(type),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<RealTopic>(yml, "topic"), ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<RealTopic>(yml, "topic"), ConfigurationException);
    }
}

/**
 * Test read RealTopic from yaml
 *
 * POSITIVE CASES:
 * - UDP
 * - TCP
 *
 * NEGATIVE CASES:
 * - Empty
 * - Incorrect tag
 */
TEST(YamlGetEntityTest, get_transport_protocol)
{
    // UDP
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_UDP_TAG),
            ADDRESS_TRANSPORT_TAG);

        TransportProtocol tp = YamlReader::get<TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG);

        ASSERT_EQ(tp, TransportProtocol::UDP);
    }

    // TCP
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_TCP_TAG),
            ADDRESS_TRANSPORT_TAG);

        TransportProtocol tp = YamlReader::get<TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG);

        ASSERT_EQ(tp, TransportProtocol::TCP);
    }

    // Empty
    {
        Yaml yml;

        ASSERT_THROW(YamlReader::get<TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG), ConfigurationException);
    }

    // Incorrect tag
    {
        Yaml yml;
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>("utcp"),
            ADDRESS_TRANSPORT_TAG);

        ASSERT_THROW(YamlReader::get<TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG), ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
