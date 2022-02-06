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
 * @file YamlReaderDiscoveryConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/YamlManager.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

#include "YamlReaderDiscoveryConfiguration.hpp"

namespace eprosima {
namespace ddsrouter {
namespace discovery {
namespace yaml {

using namespace eprosima::ddsrouter::yaml;

configuration::DDSRouterConfiguration
YamlReaderDiscoveryConfiguration::get_discovery_configuration(
        const Yaml& yml)
{
    try
    {
        // Get Discovery Server Configuration
        eprosima::ddsrouter::configuration::DiscoveryServerParticipantConfiguration discovery_participant_ =
            get_discovery_participant_configuration_(yml);

        // Check it has listening addresses
        if (discovery_participant_.listening_addresses().size() < 1)
        {
            throw ConfigurationException(
                  utils::Formatter() << "Error loading DiscoveryTool configuration:\n " <<
                    "DiscoveryTool must have listening addresses.");
        }

        /////
        // Construct object

        // Allowlist empty
        std::set<std::shared_ptr<FilterTopic>> allowlist;
        // Builtin topics empty
        std::set<std::shared_ptr<RealTopic>> builtin_topics;

        // Blocklist blocks everything
        std::set<std::shared_ptr<FilterTopic>> blocklist({std::make_shared<WildcardTopic>("*")});

        // One Participant Empty
        std::shared_ptr<eprosima::ddsrouter::configuration::ParticipantConfiguration> void_participant =
            std::make_shared<eprosima::ddsrouter::configuration::ParticipantConfiguration>(
                ParticipantId("__void_participant__"),
                ParticipantKind::VOID
            );

        // One Discovery Server Participant with the given yaml information
        std::shared_ptr<eprosima::ddsrouter::configuration::ParticipantConfiguration> discovery_participant_ptr =
            std::make_shared<eprosima::ddsrouter::configuration::DiscoveryServerParticipantConfiguration>(
                discovery_participant_);

        return configuration::DDSRouterConfiguration(
            allowlist,
            blocklist,
            builtin_topics,
            {void_participant, discovery_participant_ptr});
    }
    catch (const std::exception& e)
    {
        throw ConfigurationException(
                  utils::Formatter() << "Error loading DiscoveryTool configuration:\n " << e.what());
    }
}

configuration::DDSRouterConfiguration
YamlReaderDiscoveryConfiguration::load_discovery_configuration_from_file(
        const std::string& file_path)
{
    try
    {
        yaml::Yaml yml = yaml::YamlManager::load_file(file_path);

        // Load DDS Router Configuration
        configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReaderDiscoveryConfiguration::get_discovery_configuration(yml);

        return router_configuration;
    }
    catch (const std::exception& e)
    {
        throw ConfigurationException(
                  utils::Formatter() << "Error loading DiscoveryTool configuration from file: <" << file_path <<
                      "> :\n " << e.what());
    }
}

configuration::DiscoveryServerParticipantConfiguration
YamlReaderDiscoveryConfiguration::get_discovery_participant_configuration_(
        eprosima::ddsrouter::yaml::Yaml yml)
{
    // If yml does not have name, add it
    if (!is_tag_present(yml, ddsrouter::yaml::PARTICIPANT_NAME_TAG))
    {
        yml[PARTICIPANT_NAME_TAG] = "__discovery_participant__";
    }

    // Add Participant kind
    yml[PARTICIPANT_KIND_TAG] = "local-discovery-server";

    return get<eprosima::ddsrouter::configuration::DiscoveryServerParticipantConfiguration>(yml);
}

} /* namespace yaml */
} /* namespace discovery */
} /* namespace ddsrouter */
} /* namespace eprosima */
