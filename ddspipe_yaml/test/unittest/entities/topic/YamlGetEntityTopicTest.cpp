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

#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>
#include <ddspipe_core/types/dds/TopicQoS.hpp>
#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

#include <ddspipe_yaml/testing/generate_yaml.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;
using namespace eprosima::ddspipe::core::testing;
using namespace eprosima::ddspipe::yaml::testing;

namespace eprosima {
namespace ddspipe {
namespace yaml {
namespace test {

// Create a yaml QoS object only with reliability
void qos_to_yaml(
        Yaml& yml,
        const YamlField<bool>& reliable)
{
    // TODO: extend this for all qos
    add_field_to_yaml(yml, reliable, QOS_RELIABLE_TAG);
}

// Create a yaml Topic object with name, type and key tags
void topic_to_yaml(
        Yaml& yml,
        const YamlField<std::string>& name,
        const YamlField<std::string>& type,
        const YamlField<Yaml>& qos)
{
    add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    add_field_to_yaml(yml, qos, TOPIC_QOS_TAG);
}

// Create a yaml DdsTopic object with name, type, key and reliable tags
void real_topic_to_yaml(
        Yaml& yml,
        const YamlField<std::string>& name,
        const YamlField<std::string>& type,
        const YamlField<Yaml>& qos)
{
    add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    add_field_to_yaml(yml, qos, TOPIC_QOS_TAG);
}

// Check the values of a real topic are the expected ones
void compare_topic(
        core::types::DdsTopic topic,
        std::string name,
        std::string type,
        bool has_reliability_set = false,
        bool reliable = false)
{
    ASSERT_EQ(topic.m_topic_name, name);
    ASSERT_EQ(topic.type_name, type);

    if (has_reliability_set)
    {
        eprosima::ddspipe::core::types::ReliabilityKind expected_reliability_qos;
        if (reliable)
        {
            expected_reliability_qos = core::types::ReliabilityKind::RELIABLE;
        }
        else
        {
            expected_reliability_qos = core::types::ReliabilityKind::BEST_EFFORT;
        }

        ASSERT_EQ(topic.topic_qos.reliability_qos, expected_reliability_qos);
    }
}

// Check the values of a wildcard topic are the expected ones
void compare_wildcard_topic(
        core::types::WildcardDdsFilterTopic topic,
        std::string name,
        bool type_set,
        std::string type)
{
    ASSERT_EQ(topic.topic_name, name);

    if (type_set)
    {
        ASSERT_TRUE(topic.type_name.is_set());
        ASSERT_EQ(topic.type_name, type);
    }
}

} /* namespace test */
} /* namespace yaml */
} /* namespace ddspipe */
} /* namespace eprosima */

/**
 * Test read core::types::DdsTopic from yaml
 *
 * POSITIVE CASES:
 * - Topic Std
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
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(name),
            YamlField<std::string>(type),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        compare_topic(topic, name, type, false);
    }

    // Checks that a topic yaml object has been parsed correctly with the topic reliable tag set to true.
    // A topic configured as reliable creates RELIABLE-TRANSIENT_LOCAL RTPS Readers in order to ensure
    // that no data is lost in the information relay.
    {
        Yaml yml_qos;
        qos_to_yaml(yml_qos, YamlField<bool>(true));

        Yaml yml_topic;
        real_topic_to_yaml(
            yml_topic,
            YamlField<std::string>(name),
            YamlField<std::string>(type),
            YamlField<bool>(false),
            YamlField<Yaml>(yml_qos));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::DdsTopic topic = YamlReader::get<core::types::DdsTopic>(yml, "topic", LATEST);

        compare_topic(topic, name, type, true, true);
    }

    // Empty
    {
        Yaml yml_topic;
        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic",
                LATEST), eprosima::utils::ConfigurationException);
    }

    // Topic without name
    {
        Yaml yml_topic;
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(),
            YamlField<std::string>(type),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic",
                LATEST), eprosima::utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(name),
            YamlField<std::string>(),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::DdsTopic>(yml, "topic",
                LATEST), eprosima::utils::ConfigurationException);
    }
}

/**
 * Test read correct core::types::WildcardDdsFilterTopic from yaml
 *
 * POSITIVE CASES:
 * - Topic Std
 * - Topic without type
 */
TEST(YamlGetEntityTopicTest, get_wildcard_topic)
{
    std::string name = "topic_name";
    std::string type = "topic_type";

    // Topic Std
    {
        Yaml yml_topic;
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(name),
            YamlField<std::string>(type),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        compare_wildcard_topic(topic, name, true, type, false);
    }

    // Topic without type
    {
        Yaml yml_topic;
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(name),
            YamlField<std::string>(),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardDdsFilterTopic topic = YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                        LATEST);

        compare_wildcard_topic(topic, name, false, "*", false);
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
                LATEST), eprosima::utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        topic_to_yaml(
            yml_topic,
            YamlField<std::string>(),
            YamlField<std::string>(type),
            YamlField<bool>(),
            YamlField<Yaml>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::WildcardDdsFilterTopic>(yml, "topic",
                LATEST), eprosima::utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
