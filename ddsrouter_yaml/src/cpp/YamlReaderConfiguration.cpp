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

#include <ddsrouter_core/participants/participant/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/participants/participant/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter_core/participants/participant/configuration/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/participants/participant/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddsrouter_core/participants/participant/configuration/EchoParticipantConfiguration.hpp>
#include <ddsrouter_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>
#include <ddsrouter_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddsrouter_yaml/Yaml.hpp>
#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/YamlManager.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

using namespace eprosima::ddsrouter::core;

Configuration::Configuration (const Yaml& yml)
{
    load_ddsrouter_configuration_(yml);
}

Configuration::Configuration (const std::string& file_path)
{
    load_ddsrouter_configuration_from_file_(file_path);
}

void Configuration::load_ddsrouter_configuration_(
        const Yaml& yml)
{
    try
    {
        YamlReaderVersion version;
        // Get version if present
        if (YamlReader::is_tag_present(yml, VERSION_TAG))
        {
            version = YamlReader::get<YamlReaderVersion>(yml, VERSION_TAG, LATEST);
        }
        else
        {
            // Get default version
            version = default_yaml_version_();
            logWarning(DDSROUTER_YAML,
                    "No version of yaml configuration given. Using version " << version << " by default. " <<
                    "Add " << VERSION_TAG << " tag to your configuration in order to not break compatibility " <<
                    "in future releases.");
        }
        logInfo(DDSROUTER_YAML, "Loading DDSRouter configuration with version: " << version << ".");

        // Load DDS Router Configuration
        configuration =
                yaml::YamlReader::get<core::configuration::DDSRouterConfiguration>(yml, version);

        /////
        // Load Participants Configurations
        load_participant_configurations_(yml, version);

    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDS Router configuration from yaml:\n " << e.what());
    }
}

void Configuration::load_ddsrouter_configuration_from_file_(
        const std::string& file_path)
{
    yaml::Yaml yml;

    // Load file
    try
    {
        yml = yaml::YamlManager::load_file(file_path);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDSRouter configuration from file: <" << file_path <<
                      "> :\n " << e.what());
    }

    Configuration::load_ddsrouter_configuration_(yml);
}

void Configuration::load_participant_configurations_(
        const Yaml& yml,
        const YamlReaderVersion& version)
{
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
        // Get kind
        participants::ParticipantKind kind = YamlReader::get<participants::ParticipantKind>(conf, PARTICIPANT_KIND_TAG, version);

        // Get Participant
        std::shared_ptr<participants::ParticipantConfiguration> configuration;

        logInfo(DDSROUTER_YAML_CONFIGURATION, "Loading Participant of kind " << kind << ".");

        switch (kind)
        {
            case participants::ParticipantKind::echo:
                configuration =
                    std::make_shared<participants::EchoParticipantConfiguration>(
                        YamlReader::get<participants::EchoParticipantConfiguration>(conf, version));
                break;

            case participants::ParticipantKind::simple:
                configuration =
                    std::make_shared<participants::SimpleParticipantConfiguration>(
                        YamlReader::get<participants::SimpleParticipantConfiguration>(conf, version));
                break;

            case participants::ParticipantKind::discovery_server:
                configuration =
                    std::make_shared<participants::DiscoveryServerParticipantConfiguration>(
                        YamlReader::get<participants::DiscoveryServerParticipantConfiguration>(conf, version));
                break;

            case participants::ParticipantKind::initial_peers:
                configuration =
                    std::make_shared<participants::InitialPeersParticipantConfiguration>(
                        YamlReader::get<participants::InitialPeersParticipantConfiguration>(conf, version));
                break;

            // No default possible
        }

        participants_configurations.emplace_back(
            kind,
            std::move(configuration));
    }
}

YamlReaderVersion Configuration::default_yaml_version_()
{
    return V_1_0;
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
