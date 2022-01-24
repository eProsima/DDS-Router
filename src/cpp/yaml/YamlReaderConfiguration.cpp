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

/**
 * @file YamlReaderConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/YamlManager.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

configuration::DDSRouterConfiguration
    YamlReaderConfiguration::get_ddsrouter_configuration(const Yaml& yml)
{
    try
    {
        /////
        // Get optional allowlist
        std::set<std::shared_ptr<FilterTopic>> allowlist;
        if (YamlReader::is_tag_present(yml, ALLOWLIST_TAG))
        {
            allowlist = utils::convert_set_to_shared<FilterTopic>(
                YamlReader::get_set<WildcardTopic>(yml, ALLOWLIST_TAG));
        }

        /////
        // Get optional blocklist
        std::set<std::shared_ptr<FilterTopic>> blocklist;
        if (YamlReader::is_tag_present(yml, BLOCKLIST_TAG))
        {
            blocklist = utils::convert_set_to_shared<FilterTopic>(
                YamlReader::get_set<WildcardTopic>(yml, BLOCKLIST_TAG));
        }

        /////
        // Get optional builtin topics
        std::set<std::shared_ptr<RealTopic>> builtin_topics;
        if (YamlReader::is_tag_present(yml, BUILTIN_TAG))
        {
            builtin_topics = utils::convert_set_to_shared<RealTopic>(
                YamlReader::get_set<RealTopic>(yml, BUILTIN_TAG));
        }

        /////
        // Get participants configurations. Required field, if get_value_in_tag fail propagate exception.
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participants_configurations;
        auto participants_configurations_yml = YamlReader::get_value_in_tag(yml, COLLECTION_PARTICIPANTS_TAG);

        // Check it is a list
        if(!participants_configurations_yml.IsSequence())
        {
            throw ConfigurationException(
                utils::Formatter() <<
                "Participant configurations must be specified in an array under tag: " <<
                COLLECTION_PARTICIPANTS_TAG);
        }

        for (auto conf : participants_configurations_yml)
        {
            participants_configurations.insert(participants_yaml_factory_(conf));
        }

        /////
        // Construct object
        return configuration::DDSRouterConfiguration(
            allowlist,
            blocklist,
            builtin_topics,
            participants_configurations);
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() << "Error loading DDS Router configuration:\n " << e.what());
    }
}

std::shared_ptr<configuration::ParticipantConfiguration>
    YamlReaderConfiguration::participants_yaml_factory_(const Yaml& yml)
{
    // Kind required
    ParticipantKind kind = YamlReader::get<ParticipantKind>(yml, PARTICIPANT_KIND_TAG);

    logInfo(DDSROUTER_YAML_CONFIGURATION, "Loading Participant of kind " << kind << ".");

    switch (kind())
    {
    case ParticipantKind::ECHO:
    case ParticipantKind::DUMMY:
        return std::make_shared<configuration::ParticipantConfiguration>(
            YamlReader::get<configuration::ParticipantConfiguration>(yml));

    case ParticipantKind::SIMPLE_RTPS:
        return std::make_shared<configuration::SimpleParticipantConfiguration>(
            YamlReader::get<configuration::SimpleParticipantConfiguration>(yml));

    case ParticipantKind::LOCAL_DISCOVERY_SERVER:
    case ParticipantKind::WAN:
        return std::make_shared<configuration::DiscoveryServerParticipantConfiguration>(
            YamlReader::get<configuration::DiscoveryServerParticipantConfiguration>(yml));

    default:
        throw ConfigurationException(
            utils::Formatter() << "Unkown or non valid Participant type:" << kind << ".");
        break;
    }
}

configuration::DDSRouterConfiguration
    YamlReaderConfiguration::load_ddsrouter_configuration_from_file(const std::string& file_path)
{
    try
    {
        yaml::Yaml yml = yaml::YamlManager::load_file(file_path);

        // Load DDS Router Configuration
        configuration::DDSRouterConfiguration router_configuration =
            yaml::YamlReaderConfiguration::get_ddsrouter_configuration(yml);

        return router_configuration;
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() << "Error loading DDSRouter configuration from file: <" << file_path <<
            "> :\n " << e.what());
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
