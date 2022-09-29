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

#include <ddsrouter_core/types/address/Address.hpp>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>
#include <ddsrouter_core/types/dds/TopicQoS.hpp>
#include <ddsrouter_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

#include "../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace test {

// Create a yaml QoS object only with reliability
void qos_to_yaml(
        Yaml& yml,
        const test::YamlField<bool>& reliable)
{
    // TODO: extend this for all qos
    test::add_field_to_yaml(yml, reliable, QOS_RELIABLE_TAG);
}

// Create a yaml Topic object with name, type and key tags
void topic_to_yaml(
        Yaml& yml,
        const test::YamlField<std::string>& name,
        const test::YamlField<std::string>& type,
        const test::YamlField<bool>& keyed,
        const test::YamlField<Yaml>& qos)
{
    test::add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    test::add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    test::add_field_to_yaml(yml, keyed, TOPIC_KIND_TAG);
    test::add_field_to_yaml(yml, qos, TOPIC_QOS_TAG);
}

// Create a yaml DdsTopic object with name, type, key and reliable tags
void real_topic_to_yaml(
        Yaml& yml,
        const test::YamlField<std::string>& name,
        const test::YamlField<std::string>& type,
        const test::YamlField<bool>& keyed,
        const test::YamlField<Yaml>& qos)
{
    test::add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    test::add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    test::add_field_to_yaml(yml, keyed, TOPIC_KIND_TAG);
    test::add_field_to_yaml(yml, qos, TOPIC_QOS_TAG);
}

// Check the values of a real topic are the expected ones
void compare_topic(
        core::types::DdsTopic topic,
        std::string name,
        std::string type,
        bool keyed,
        bool has_reliability_set = false,
        bool reliable = false)
{
    ASSERT_EQ(topic.topic_name, name);
    ASSERT_EQ(topic.type_name, type);
    ASSERT_EQ(topic.keyed, keyed);

    if (has_reliability_set)
    {
        eprosima::ddsrouter::core::types::ReliabilityKind expected_reliability_qos;
        if (reliable)
        {
            expected_reliability_qos = core::types::ReliabilityKind::RELIABLE;
        }
        else
        {
            expected_reliability_qos = core::types::ReliabilityKind::BEST_EFFORT;
        }

        ASSERT_EQ(topic.topic_qos.get_reference().reliability_qos, expected_reliability_qos);
    }
}

// Check the values of a wildcard topic are the expected ones
void compare_wildcard_topic(
        core::types::WildcardDdsFilterTopic topic,
        std::string name,
        bool type_set,
        std::string type,
        bool key_set,
        bool keyed)
{
    ASSERT_EQ(topic.topic_name, name);

    if (type_set)
    {
        ASSERT_TRUE(topic.type_name.is_set());
        ASSERT_EQ(topic.type_name.get_reference(), type);
    }

    if (key_set)
    {
        ASSERT_TRUE(topic.keyed.is_set());
        ASSERT_EQ(topic.keyed.get_reference(), keyed);
    }
}

} /* namespace test */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test read core::types::DdsTopic from yaml
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
TEST(YamlGetEntityTopicTest, get_real_topic)
{
    std::string name = "topic_name";
    std::string type = "topic_type";

    // Topic Std
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false); // By default no keyed
    }

    // Topic with key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(true),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, true);
    }

    // Topic with no key
    {
        Yaml yml_topic;
        test::real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false);
    }

    // Checks that a topic yaml object has been parsed correctly with the topic reliable tag set to true.
    // A topic configured as reliable creates RELIABLE-TRANSIENT_LOCAL RTPS Readers in order to ensure
    // that no data is lost in the information relay.
    {
        Yaml yml_qos;
        test::qos_to_yaml(yml_qos, test::YamlField<bool>(true));

        Yaml yml_topic;
        test::real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false),
            test::YamlField<Yaml>(yml_qos));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false, true, true); // By default no keyed
    }

    // Empty
    {
        Yaml yml_topic;
        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }

    // Topic without name
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }
}

/**
 * Test read correct core::types::WildcardDdsFilterTopic from yaml
 *
 * POSITIVE CASES:
 * - Topic Std
 * - Topic without type
 * - Topic with key
 * - Topic with no key
 * - Topic with key without type
 */
TEST(YamlGetEntityTopicTest, get_wildcard_topic)
{
    std::string name = "topic_name";
    std::string type = "topic_type";

    // Topic Std
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        test::compare_wildcard_topic(topic, name, true, type, false, false); // By default no keyed
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        test::compare_wildcard_topic(topic, name, false, "*", false, false); // By default no keyed
    }

    // Topic with key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(true),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        test::compare_wildcard_topic(topic, name, true, type, true, true);
    }

    // Topic with no key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        test::compare_wildcard_topic(topic, name, true, type, true, false);
    }

    // Topic with key without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>(true),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        test::compare_wildcard_topic(topic, name, false, "*", true, true);
    }
}

/**
 * Test read correct core::types::WildcardDdsFilterTopic from yaml
 *
 * NEGATIVE CASES:
 * - empty
 * - without name
 */
TEST(YamlGetEntityTopicTest, get_wildcard_topic_negative)
{
    std::string name = "topic_name";
    std::string type = "topic_type";

    // empty
    {
        Yaml yml_topic;
        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                LATEST), utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(),
            test::YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                LATEST), utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
