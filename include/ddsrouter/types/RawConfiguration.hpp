// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RawConfiguration.hpp
 */

#ifndef _DDSROUTER_TYPES_RAWCONFIGURATION_HPP_
#define _DDSROUTER_TYPES_RAWCONFIGURATION_HPP_

#include <yaml-cpp/yaml.h>

namespace eprosima {
namespace ddsrouter {

/**
 * Configuration is in dictionary format
 *
 * YAML spec: https://yaml.org/spec/1.2.2/
 *
 * @note: It is not legal to repeat keys in a YAML
 */
using RawConfiguration = YAML::Node;
using RawConfigurationType = YAML::NodeType;

/**
 * @brief Load a \c RawConfiguration object form a .yaml file
 *
 * @param [in] file_path : path of file to load
 * @return yaml in RawConfiguration format contained in file_path
 *
 * @throw \c ConfigurationException in case the file does not exist or it is not a correct yaml
 */
RawConfiguration load_configuration_from_file(
        const std::string& file_path);

// TODO: add way to compare equality and not identity of yaml

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_RAWCONFIGURATION_HPP_ */
