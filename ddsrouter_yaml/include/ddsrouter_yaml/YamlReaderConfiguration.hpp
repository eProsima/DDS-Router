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

#pragma once

#include <ddspipe_yaml/Yaml.hpp>
#include <ddspipe_yaml/YamlReader.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>

#include <ddsrouter_yaml/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/**
 * @brief Class that encapsulates specific methods to get a full DdsRouter Configuration from a yaml node.
 *
 * TODO: Add version configuration so it could load different versions
 */
class DDSROUTER_YAML_DllAPI YamlReaderConfiguration
{
public:

    static ddsrouter::core::DdsRouterConfiguration load_ddsrouter_configuration(
            const Yaml& yml);

    static ddsrouter::core::DdsRouterConfiguration load_ddsrouter_configuration_from_file(
            const std::string& file_path);

protected:

    static ddspipe::yaml::YamlReaderVersion default_yaml_version();
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
