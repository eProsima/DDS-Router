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
 * @file YamlReaderConfiguration.hpp
 */

#ifndef _DDSROUTERYAML_YAMLREADERCONFIGURATION_HPP_
#define _DDSROUTERYAML_YAMLREADERCONFIGURATION_HPP_

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>

#include <ddsrouter_yaml/library/library_dll.h>
#include <ddsrouter_yaml/YamlReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/**
 * @brief Class that encapsulates specific methods to get a full DDSRouter Configuration from a yaml node.
 *
 * TODO: Add version configuration so it could load different versions
 */
class DDSROUTER_YAML_DllAPI YamlReaderConfiguration : public YamlReader
{
public:

    static core::configuration::DDSRouterConfiguration load_ddsrouter_configuration(
            const Yaml& yml);

    static core::configuration::DDSRouterConfiguration load_ddsrouter_configuration_from_file(
            const std::string& file_path);

protected:

    static YamlReaderVersion load_yaml_version(
            const Yaml& yml);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERYAML_YAMLREADERCONFIGURATION_HPP_ */
