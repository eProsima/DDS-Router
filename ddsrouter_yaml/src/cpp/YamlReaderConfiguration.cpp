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

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/types/topic/FilterTopic.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>

#include <ddsrouter_yaml/Yaml.hpp>
#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/YamlManager.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

using namespace eprosima::ddsrouter::core;

core::configuration::DDSRouterConfiguration
YamlReaderConfiguration::get_ddsrouter_configuration(
        const Yaml& yml)
{
    try
    {
        /////
        // Get optional allowlist
        std::set<std::shared_ptr<types::FilterTopic>> allowlist;
        if (YamlReader::is_tag_present(yml, ALLOWLIST_TAG))
        {
            allowlist = utils::convert_set_to_shared<types::FilterTopic>(
                YamlReader::get_set<types::WildcardTopic>(yml, ALLOWLIST_TAG));
        }

        /////
        // Get optional blocklist
        std::set<std::shared_ptr<types::FilterTopic>> blocklist;
        if (YamlReader::is_tag_present(yml, BLOCKLIST_TAG))
        {
            blocklist = utils::convert_set_to_shared<types::FilterTopic>(
                YamlReader::get_set<types::WildcardTopic>(yml, BLOCKLIST_TAG));
        }

        /////
        // Get optional builtin topics
        std::set<std::shared_ptr<types::RealTopic>> builtin_topics;
        if (YamlReader::is_tag_present(yml, BUILTIN_TAG))
        {
            builtin_topics = utils::convert_set_to_shared<types::RealTopic>(
                YamlReader::get_set<types::RealTopic>(yml, BUILTIN_TAG));
        }

        /////
        // Get participants configurations. Required field, if get_value_in_tag fail propagate exception.
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participants_configurations;
        auto participants_configurations_yml = YamlReader::get_value_in_tag(yml, COLLECTION_PARTICIPANTS_TAG);

        // Check it is a list
        if (!participants_configurations_yml.IsSequence())
        {
            throw utils::ConfigurationException(
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
        return core::configuration::DDSRouterConfiguration(
            allowlist,
            blocklist,
            builtin_topics,
            participants_configurations);
    }
    catch (const std::exception& e)
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDS Router configuration:\n " << e.what());
    }
}

std::shared_ptr<core::configuration::ParticipantConfiguration>
YamlReaderConfiguration::participants_yaml_factory_(
        const Yaml& yml)
{
    // Kind required
    types::ParticipantKind kind = YamlReader::get<types::ParticipantKind>(yml, PARTICIPANT_KIND_TAG);

    logInfo(DDSROUTER_YAML_CONFIGURATION, "Loading Participant of kind " << kind << ".");

    switch (kind())
    {
        case types::ParticipantKind::VOID:
        case types::ParticipantKind::ECHO:
        case types::ParticipantKind::DUMMY:
            return std::make_shared<core::configuration::ParticipantConfiguration>(
                YamlReader::get<core::configuration::ParticipantConfiguration>(yml));

        case types::ParticipantKind::SIMPLE_RTPS:
            return std::make_shared<core::configuration::SimpleParticipantConfiguration>(
                YamlReader::get<core::configuration::SimpleParticipantConfiguration>(yml));

        case types::ParticipantKind::LOCAL_DISCOVERY_SERVER:
        case types::ParticipantKind::WAN:
            return std::make_shared<core::configuration::DiscoveryServerParticipantConfiguration>(
                YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml));

        default:
            throw utils::ConfigurationException(
                      utils::Formatter() << "Unkown or non valid Participant kind:" << kind << ".");
            break;
    }
}

core::configuration::DDSRouterConfiguration
YamlReaderConfiguration::load_ddsrouter_configuration_from_file(
        const std::string& file_path)
{
    try
    {
        yaml::Yaml yml = yaml::YamlManager::load_file(file_path);

        // Load DDS Router Configuration
        core::configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReaderConfiguration::get_ddsrouter_configuration(yml);

        return router_configuration;
    }
    catch (const std::exception& e)
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDSRouter configuration from file: <" << file_path <<
                      "> :\n " << e.what());
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
