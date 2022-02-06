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
 * @file YamlReaderDiscoveryConfiguration.hpp
 */

#ifndef _DDSROUTER_TOOL_CPP_DISCOVERY_YAML_YAMLREADERDISCOVERYCONFIGURATION_HPP_
#define _DDSROUTER_TOOL_CPP_DISCOVERY_YAML_YAMLREADERDISCOVERYCONFIGURATION_HPP_

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace discovery {
namespace yaml {

/**
 * @brief Class that encapsulates specific methods to get a full DDSRouter Configuration from a yaml node.
 *
 * TODO: Add version configuration so it could load different versions
 */
class YamlReaderDiscoveryConfiguration : public eprosima::ddsrouter::yaml::YamlReader
{
public:

    static configuration::DDSRouterConfiguration get_discovery_configuration(
            const eprosima::ddsrouter::yaml::Yaml& yml);

    static configuration::DDSRouterConfiguration load_discovery_configuration_from_file(
            const std::string& file_path);

protected:

    static configuration::DiscoveryServerParticipantConfiguration get_discovery_participant_configuration_(
            eprosima::ddsrouter::yaml::Yaml yml);

};

} /* namespace yaml */
} /* namespace discovery */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TOOL_CPP_DISCOVERY_YAML_YAMLREADERDISCOVERYCONFIGURATION_HPP_ */
