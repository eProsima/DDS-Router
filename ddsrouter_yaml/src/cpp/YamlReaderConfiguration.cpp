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

core::configuration::DDSRouterConfiguration
YamlReaderConfiguration::load_ddsrouter_configuration(
        const Yaml& yml)
{
    try
    {
        YamlReaderVersion version;
        // Get version if present
        if (is_tag_present(yml, VERSION_TAG))
        {
            version = get<YamlReaderVersion>(yml, VERSION_TAG, LATEST);
        }
        else
        {
            // Get default version
            version = default_yaml_version();
            logWarning(DDSROUTER_YAML,
                    "No version of yaml configuration given. Using version " << version << " by default. " <<
                    "Add " << VERSION_TAG << " tag to your configuration in order to not break compatibility " <<
                    "in future releases.");
        }
        logInfo(DDSROUTER_YAML, "Loading DDSRouter configuration with version: " << version << ".");

        // Load DDS Router Configuration
        core::configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReader::get<core::configuration::DDSRouterConfiguration>(yml, version);

        return router_configuration;
    }
    catch (const std::exception& e)
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDS Router configuration from yaml:\n " << e.what());
    }
}

core::configuration::DDSRouterConfiguration
YamlReaderConfiguration::load_ddsrouter_configuration_from_file(
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
        throw utils::ConfigurationException(
                  utils::Formatter() << "Error loading DDSRouter configuration from file: <" << file_path <<
                      "> :\n " << e.what());
    }

    return YamlReaderConfiguration::load_ddsrouter_configuration(yml);
}

YamlReaderVersion YamlReaderConfiguration::default_yaml_version()
{
    return V_1_0;
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
