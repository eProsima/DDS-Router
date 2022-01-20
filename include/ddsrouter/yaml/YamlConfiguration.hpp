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

#ifndef _DDSROUTER_YAML_YAMLCONFIGURATION_HPP_
#define _DDSROUTER_YAML_YAMLCONFIGURATION_HPP_

#include <ddsrouter/yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

class YamlConfiguration
{
public:

    template <typename T>
    static T get(const Yaml& yml);

    template <typename T>
    static T get(const Yaml& yml, const std::string& tag);

    template <typename T>
    static T get_scalar(const Yaml& yml);

    template <typename T>
    static T get_scalar(const Yaml& yml, const std::string& tag);

    template <typename T>
    static std::list<T> get_list(const Yaml& yml, const std::string& tag);

    template <typename T>
    static std::list<T> get_list(const Yaml& yml);

    template <typename T>
    static std::set<T> get_set(const Yaml& yml, const std::string& tag);

    template <typename T>
    static T get_enumeration(const Yaml& yml, const std::map<std::string, T>& enum_values);

    template <typename T>
    static T get_enumeration(const Yaml& yml, const std::string& tag, const std::map<std::string, T>& enum_values);

    template <typename T>
    static bool is_present(const Yaml& yml);

    static bool is_tag_present(const Yaml& yml, const std::string& tag);

    static Yaml get_value_in_tag(const Yaml& yml, const std::string& tag);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/yaml/implementations/YamlConfiguration.ipp>

#endif /* _DDSROUTER_YAML_YAMLCONFIGURATION_HPP_ */
