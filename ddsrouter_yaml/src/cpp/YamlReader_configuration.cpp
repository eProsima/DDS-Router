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

#include <set>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/XmlParticipantConfiguration.hpp>

#include <ddspipe_yaml/yaml_configuration_tags.hpp>
#include <ddspipe_yaml/Yaml.hpp>
#include <ddspipe_yaml/YamlManager.hpp>
#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/YamlValidator.hpp>

#include <ddspipe_core/configuration/DdsPipeConfiguration.hpp>
#include <ddspipe_core/configuration/MonitorConfiguration.hpp>
#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>

#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>

namespace eprosima {
namespace ddspipe {
namespace yaml {

template <>
bool YamlValidator::validate<core::MonitorConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion& /* version */)
{
    // The method is rewritten to provide a specific validation of the DDS Router's MonitorConfiguration:
    // i.e. the DDS Router's MonitorConfiguration doesn't have a status.
    static const std::set<TagType> tags{
        MONITOR_DOMAIN_TAG,
        MONITOR_TOPICS_TAG};

    return YamlValidator::validate_tags(yml, tags);
}

template <>
void YamlReader::fill(
        ddsrouter::core::SpecsConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    /////
    // Get optional number of threads
    if (YamlReader::is_tag_present(yml, NUMBER_THREADS_TAG))
    {
        object.number_of_threads = YamlReader::get<unsigned int>(yml, NUMBER_THREADS_TAG, version);
    }

    /////
    // Get optional remove unused entities tag
    if (YamlReader::is_tag_present(yml, REMOVE_UNUSED_ENTITIES_TAG))
    {
        object.remove_unused_entities = YamlReader::get<bool>(yml, REMOVE_UNUSED_ENTITIES_TAG, version);
    }

    // Optional Topic QoS
    if (is_tag_present(yml, SPECS_QOS_TAG))
    {
        object.topic_qos = YamlReader::get<core::types::TopicQoS>(yml, SPECS_QOS_TAG, version);
        core::types::TopicQoS::default_topic_qos.set_value(object.topic_qos);
    }

    /////
    // Get optional discovery trigger tag
    if (YamlReader::is_tag_present(yml, DISCOVERY_TRIGGER_TAG))
    {
        const std::string discovery_trigger = YamlReader::get<std::string>(yml, DISCOVERY_TRIGGER_TAG, version);

        std::string discovery_trigger_caps = discovery_trigger;
        utils::to_uppercase(discovery_trigger_caps);

        const bool ret_code = ddspipe::core::string_to_enumeration(discovery_trigger_caps, object.discovery_trigger);

        if (!ret_code)
        {
            throw eprosima::utils::ConfigurationException(
                      utils::Formatter() << "The discovery-trigger " << discovery_trigger << " is not valid.");
        }
    }

    /////
    // Get optional Log Configuration
    if (YamlReader::is_tag_present(yml, LOG_CONFIGURATION_TAG))
    {
        object.log_configuration = YamlReader::get<ddspipe::core::DdsPipeLogConfiguration>(yml, LOG_CONFIGURATION_TAG,
                        version);
    }

    /////
    // Get optional monitor tag
    if (YamlReader::is_tag_present(yml, MONITOR_TAG))
    {
        object.monitor_configuration = YamlReader::get<core::MonitorConfiguration>(yml, MONITOR_TAG, version);
    }
}

template <>
bool YamlValidator::validate<ddsrouter::core::SpecsConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion& /* version */)
{
    static const std::set<TagType> tags{
        NUMBER_THREADS_TAG,
        REMOVE_UNUSED_ENTITIES_TAG,
        SPECS_QOS_TAG,
        DISCOVERY_TRIGGER_TAG,
        LOG_CONFIGURATION_TAG,
        MONITOR_TAG};

    return YamlValidator::validate_tags(yml, tags);
}

template <>
ddsrouter::core::SpecsConfiguration YamlReader::get<ddsrouter::core::SpecsConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    YamlValidator::validate<ddsrouter::core::SpecsConfiguration>(yml, version);

    ddsrouter::core::SpecsConfiguration object;
    fill<ddsrouter::core::SpecsConfiguration>(object, yml, version);
    return object;
}

template <>
ddsrouter::core::types::ParticipantKind YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Domain id required
    return get_enumeration_from_builder<ddsrouter::core::types::ParticipantKind>(yml,
                   *ddsrouter::core::types::ParticipantKindBuilder::get_instance());
}

template <>
std::shared_ptr<participants::ParticipantConfiguration>
YamlReader::get<std::shared_ptr<participants::ParticipantConfiguration>>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Kind required
    ddsrouter::core::types::ParticipantKind kind = YamlReader::get<ddsrouter::core::types::ParticipantKind>(yml,
                    PARTICIPANT_KIND_TAG,
                    version);

    logInfo(DDSROUTER_YAML_CONFIGURATION, "Loading Participant of kind " << kind << ".");

    switch (kind)
    {
        case ddsrouter::core::types::ParticipantKind::echo:
            return std::make_shared<participants::EchoParticipantConfiguration>(
                YamlReader::get<participants::EchoParticipantConfiguration>(yml, version));

        case ddsrouter::core::types::ParticipantKind::simple:
            return std::make_shared<participants::SimpleParticipantConfiguration>(
                YamlReader::get<participants::SimpleParticipantConfiguration>(yml, version));

        case ddsrouter::core::types::ParticipantKind::discovery_server:
            return std::make_shared<participants::DiscoveryServerParticipantConfiguration>(
                YamlReader::get<participants::DiscoveryServerParticipantConfiguration>(yml, version));

        case ddsrouter::core::types::ParticipantKind::initial_peers:
            return std::make_shared<participants::InitialPeersParticipantConfiguration>(
                YamlReader::get<participants::InitialPeersParticipantConfiguration>(yml, version));

        case ddsrouter::core::types::ParticipantKind::xml:
            return std::make_shared<participants::XmlParticipantConfiguration>(
                YamlReader::get<participants::XmlParticipantConfiguration>(yml, version));

        default:
            // Non recheable code
            throw eprosima::utils::ConfigurationException(
                      utils::Formatter() << "Unkown or non valid Participant kind: " << kind << ".");
    }
}

template <>
void YamlReader::fill(
        core::DdsPipeConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    /////
    // Get optional allowlist
    if (YamlReader::is_tag_present(yml, ALLOWLIST_TAG))
    {
        auto allowlist_set = YamlReader::get_set<core::types::WildcardDdsFilterTopic>(yml, ALLOWLIST_TAG, version);
        for (auto const& wild_topic : allowlist_set)
        {
            auto new_topic = utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(wild_topic);
            object.allowlist.insert(new_topic);
        }
    }

    /////
    // Get optional blocklist
    if (YamlReader::is_tag_present(yml, BLOCKLIST_TAG))
    {
        auto blocklist_set = YamlReader::get_set<core::types::WildcardDdsFilterTopic>(yml, BLOCKLIST_TAG, version);
        for (auto const& wild_topic : blocklist_set)
        {
            auto new_topic = utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(wild_topic);
            object.blocklist.insert(new_topic);
        }
    }

    /////
    // Get optional builtin topics
    if (YamlReader::is_tag_present(yml, BUILTIN_TAG))
    {
        object.builtin_topics = YamlReader::get_set<utils::Heritable<core::types::DistributedTopic>>(yml,
                        BUILTIN_TAG,
                        version);
    }

    /////
    // Get optional routes
    if (YamlReader::is_tag_present(yml, ROUTES_TAG))
    {
        object.routes = YamlReader::get<core::RoutesConfiguration>(yml, ROUTES_TAG, version);
    }

    /////
    // Get optional topic routes
    if (YamlReader::is_tag_present(yml, TOPIC_ROUTES_TAG))
    {
        object.topic_routes = YamlReader::get<core::TopicRoutesConfiguration>(yml, TOPIC_ROUTES_TAG, version);
    }

    /////
    // Get optional topics
    if (YamlReader::is_tag_present(yml, TOPICS_TAG))
    {
        const auto& manual_topics = YamlReader::get_list<core::types::ManualTopic>(yml, TOPICS_TAG, version);
        object.manual_topics = std::vector<core::types::ManualTopic>(manual_topics.begin(), manual_topics.end());
    }
}

template <>
core::DdsPipeConfiguration YamlReader::get<core::DdsPipeConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // The DdsPipeConfiguration's fields don't correspond to a YAML section and, therefore, can't be validated.
    // Instead, they are validated in the methods above (most of them in the DdsRouterConfiguration).

    core::DdsPipeConfiguration object;
    fill<core::DdsPipeConfiguration>(object, yml, version);
    return object;
}

template <>
void YamlReader::fill(
        ddsrouter::core::DdsRouterConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    /////
    // Get participants configurations. Required field, if get_value_in_tag fail propagate exception.
    auto participants_configurations_yml = YamlReader::get_value_in_tag(yml, COLLECTION_PARTICIPANTS_TAG);

    // TODO do it in a single instruction
    // Check it is a list
    if (!participants_configurations_yml.IsSequence())
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() <<
                      "Participant configurations must be specified in an array under tag: " <<
                      COLLECTION_PARTICIPANTS_TAG);
    }

    for (auto conf : participants_configurations_yml)
    {
        ddsrouter::core::types::ParticipantKind kind =
                YamlReader::get<ddsrouter::core::types::ParticipantKind>(conf, PARTICIPANT_KIND_TAG, version);

        object.participants_configurations.insert(
                    {
                        kind,
                        YamlReader::get<std::shared_ptr<participants::ParticipantConfiguration>>(conf, version)
                    }
            );
    }

    /////
    // Get optional specs configuration
    if (YamlReader::is_tag_present(yml, SPECS_TAG))
    {
        object.advanced_options = get<ddsrouter::core::SpecsConfiguration>(YamlReader::get_value_in_tag(yml,
                        SPECS_TAG),
                        version);
    }

    // DDS Pipe Configuration
    object.ddspipe_configuration = YamlReader::get<core::DdsPipeConfiguration>(yml, version);

    /* NOTE
     *
     * remove_unused_entities and discovery_trigger are attributes of SpecsConfiguration because they are
     * under the tag specs, but since they are used in the DdsPipe, we have two choices: copying them to the
     * DdsPipeConfiguration, as we are doing, or refilling the SpecsConfiguraton in the DdsPipeConfiguration fill
     * and taking both attributes from there.
     */
    object.ddspipe_configuration.remove_unused_entities = object.advanced_options.remove_unused_entities;
    object.ddspipe_configuration.discovery_trigger = object.advanced_options.discovery_trigger;
    object.ddspipe_configuration.log_configuration = object.advanced_options.log_configuration;

    /////
    // Get optional xml configuration
    if (YamlReader::is_tag_present(yml, XML_TAG))
    {
        object.xml_configuration = YamlReader::get<participants::XmlHandlerConfiguration>(yml, XML_TAG, version);
    }
}

template <>
DDSPIPE_YAML_DllAPI
bool YamlValidator::validate<ddsrouter::core::DdsRouterConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion& /* version */)
{
    static const std::set<TagType> tags{
        VERSION_TAG,
        COLLECTION_PARTICIPANTS_TAG,
        XML_TAG,
        ALLOWLIST_TAG,
        BLOCKLIST_TAG,
        BUILTIN_TAG,
        ROUTES_TAG,
        TOPIC_ROUTES_TAG,
        TOPICS_TAG,
        SPECS_TAG};

    return YamlValidator::validate_tags(yml, tags);
}

template <>
ddsrouter::core::DdsRouterConfiguration YamlReader::get<ddsrouter::core::DdsRouterConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    YamlValidator::validate<ddsrouter::core::DdsRouterConfiguration>(yml, version);

    ddsrouter::core::DdsRouterConfiguration object;
    fill<ddsrouter::core::DdsRouterConfiguration>(object, yml, version);
    return object;
}

} // namespace yaml
} // namespace ddspipe
} // namespace eprosima
