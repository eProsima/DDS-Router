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
 * @file DDSRouterConfiguration.hpp
 */

#ifndef _DDSROUTER_YAML_YAMLREADERCONFIGURATION_HPP_
#define _DDSROUTER_YAML_YAMLREADERCONFIGURATION_HPP_

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/**
 * @brief Class that encapsulate specific methods to get a full DDSRouter Configuration from a yaml node.
 */
class YamlReaderConfiguration : public YamlReader
{
public:

    static configuration::DDSRouterConfiguration get_ddsrouter_configuration(const Yaml& yml);

    static configuration::DDSRouterConfiguration load_ddsrouter_configuration_from_file(const std::string& file_path);

protected:

    static std::shared_ptr<configuration::ParticipantConfiguration>
        participants_yaml_factory_(const Yaml& yml);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_YAML_YAMLREADERCONFIGURATION_HPP_ */
