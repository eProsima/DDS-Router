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
 * @file YamlReader.hpp
 */

#ifndef _DDSROUTERYAML_YAMLREADER_HPP_
#define _DDSROUTERYAML_YAMLREADER_HPP_

#include <ddsrouter_yaml/library/library_dll.h>
#include <ddsrouter_yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

enum YamlReaderVersion
{
    V_1_0,
    V_2_0,
    LATEST,
};

using TagType = std::string;

/**
 * @brief Class that encapsulates methods to get values from Yaml Node.
 *
 * TODO: Check if default arguments are needed. They may not be, as version should always be given
 */
class DDSROUTER_YAML_DllAPI YamlReader
{
public:

    template <typename T>
    static T get(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version = LATEST);

    template <typename T>
    static std::list<T> get_list(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version = LATEST);

    template <typename T>
    static std::set<T> get_set(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version = LATEST);

    static bool is_tag_present(
            const Yaml& yml,
            const TagType& tag);

protected:

    static Yaml get_value_in_tag(
            const Yaml& yml,
            const TagType& tag);

    template <typename T>
    static T get(
            const Yaml& yml,
            const YamlReaderVersion version = LATEST);

    template <typename T>
    static std::list<T> get_list(
            const Yaml& yml,
            const YamlReaderVersion version = LATEST);

    template <typename T>
    static T get_scalar(
            const Yaml& yml);

    template <typename T>
    static T get_scalar(
            const Yaml& yml,
            const TagType& tag);

    template <typename T>
    static T get_enumeration(
            const Yaml& yml,
            const std::map<TagType, T>& enum_values);

    template <typename T>
    static T get_enumeration(
            const Yaml& yml,
            const TagType& tag,
            const std::map<TagType, T>& enum_values);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_yaml/impl/YamlReader.ipp>

#endif /* _DDSROUTERYAML_YAMLREADER_HPP_ */
