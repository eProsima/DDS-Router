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
 * @file YamlManager.hpp
 */

#ifndef _DDSROUTERYAML_YAMLMANAGER_HPP_
#define _DDSROUTERYAML_YAMLMANAGER_HPP_

#include <ddsrouter_yaml/library/library_dll.h>
#include <ddsrouter_yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/**
 * Class that manages generic methods related with yaml load and yaml validation.
 */
class DDSROUTER_YAML_DllAPI YamlManager
{
public:

    static Yaml load_file(
            const std::string& file_path);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERYAML_YAMLMANAGER_HPP_ */
