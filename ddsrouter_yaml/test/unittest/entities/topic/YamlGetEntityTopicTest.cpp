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
#include <ddsrouter_core/types/topic/RealTopic.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

#include "../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace test {

// Create a yaml Topic object with name, type and key tags
void topic_to_yaml(
        Yaml& yml,
        const test::YamlField<std::string>& name,
        const test::YamlField<std::string>& type,
        const test::YamlField<bool>& keyed)
{
    test::add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    test::add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    test::add_field_to_yaml(yml, keyed, TOPIC_KIND_TAG);
}

// Create a yaml RealTopic object with name, type, key and reliable tags
void real_topic_to_yaml(
        Yaml& yml,
        const test::YamlField<std::string>& name,
        const test::YamlField<std::string>& type,
        const test::YamlField<bool>& keyed,
        const test::YamlField<bool>& reliable)
{
    test::add_field_to_yaml(yml, name, TOPIC_NAME_TAG);
    test::add_field_to_yaml(yml, type, TOPIC_TYPE_NAME_TAG);
    test::add_field_to_yaml(yml, keyed, TOPIC_KIND_TAG);
    test::add_field_to_yaml(yml, reliable, TOPIC_RELIABLE_TAG);
}

// Check the values of a real topic are the expected ones
void compare_topic(
        core::types::RealTopic topic,
        std::string name,
        std::string type,
        bool keyed,
        bool reliable = false)
{
    ASSERT_EQ(topic.topic_name(), name);
    ASSERT_EQ(topic.topic_type(), type);
    ASSERT_EQ(topic.topic_with_key(), keyed);
    ASSERT_EQ(topic.topic_reliable(), reliable);
}

// Check the values of a wildcard topic are the expected ones
void compare_wildcard_topic(
        core::types::WildcardTopic topic,
        std::string name,
        std::string type,
        bool key_set,
        bool keyed)
{
    ASSERT_EQ(topic.topic_name(), name);
    ASSERT_EQ(topic.topic_type(), type);
    ASSERT_EQ(topic.has_keyed_set(), key_set);
    ASSERT_EQ(topic.topic_with_key(), keyed);
}

} /* namespace test */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test read core::types::RealTopic from yaml
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
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::RealTopic topic = YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false); // By default no keyed
    }

    // Topic with key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(true));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::RealTopic topic = YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, true);
    }

    // Topic with no key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::RealTopic topic = YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false);
    }

    // Topic reliable with no key
    {
        Yaml yml_topic;
        test::real_topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false),
            test::YamlField<bool>(true));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::RealTopic topic = YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST);

        test::compare_topic(topic, name, type, false, true); // By default no keyed
    }

    // Empty
    {
        Yaml yml_topic;
        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }

    // Topic without name
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(),
            test::YamlField<std::string>(type),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::RealTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }
}

/**
 * Test read correct core::types::WildcardTopic from yaml
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
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardTopic topic = YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST);

        test::compare_wildcard_topic(topic, name, type, false, false); // By default no keyed
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardTopic topic = YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST);

        test::compare_wildcard_topic(topic, name, "*", false, false); // By default no keyed
    }

    // Topic with key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(true));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardTopic topic = YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST);

        test::compare_wildcard_topic(topic, name, type, true, true);
    }

    // Topic with no key
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(type),
            test::YamlField<bool>(false));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardTopic topic = YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST);

        test::compare_wildcard_topic(topic, name, type, true, false);
    }

    // Topic with key without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(name),
            test::YamlField<std::string>(),
            test::YamlField<bool>(true));

        Yaml yml;
        yml["topic"] = yml_topic;

        core::types::WildcardTopic topic = YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST);

        test::compare_wildcard_topic(topic, name, "*", true, true);
    }
}

/**
 * Test read correct core::types::WildcardTopic from yaml
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

        ASSERT_THROW(YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }

    // Topic without type
    {
        Yaml yml_topic;
        test::topic_to_yaml(
            yml_topic,
            test::YamlField<std::string>(),
            test::YamlField<std::string>(type),
            test::YamlField<bool>());

        Yaml yml;
        yml["topic"] = yml_topic;

        ASSERT_THROW(YamlReader::get<core::types::WildcardTopic>(yml, "topic", LATEST), utils::ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
