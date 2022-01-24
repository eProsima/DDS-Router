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
 * @file YamlReader.ipp
 *
 */

#ifndef _DDSROUTER_YAML_YAMLREADER_IPP_
#define _DDSROUTER_YAML_YAMLREADER_IPP_

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/macros.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

template <typename T>
T YamlReader::get(const Yaml& yml, const std::string& tag)
{
    // ATTENTION: This try catch can be avoided, it is only used to add verbose information
    try
    {
        return get<T>(get_value_in_tag(yml, tag));
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() <<
            "Error getting required value of type <" << TYPE_NAME(T) <<
            "> in tag <" << tag << "> :\n " << e.what());
    }

    utils::tsnh(utils::Formatter() << "Impossible to arrive to this point.");
    return get<T>(yml); // Unrecheable code
}

template <typename T>
T YamlReader::get_scalar(const Yaml& yml, const std::string& tag)
{
    return get_scalar<T>(get_value_in_tag(yml, tag));
}

template <typename T>
T YamlReader::get_scalar(const Yaml& yml)
{
    if (!yml.IsScalar())
    {
        throw ConfigurationException(
            utils::Formatter() <<
            "Trying to read a primitive value of type <" << TYPE_NAME(T) << "> from a non scalar yaml.");
    }

    try
    {
        return yml.as<T>();
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() <<
            "Incorrect format for primitive value, expected <" << TYPE_NAME(T) << ">:\n " << e.what());
    }
}

template <typename T>
std::list<T> YamlReader::get_list(const Yaml& yml, const std::string& tag)
{
    return get_list<T>(get_value_in_tag(yml, tag));
}

template <typename T>
std::list<T> YamlReader::get_list(const Yaml& yml)
{
    if (!yml.IsSequence())
    {
        throw ConfigurationException(
            utils::Formatter() << "Incorrect format, yaml Sequence expected.");
    }

    std::list<T> result;

    try
    {
        for (Yaml element : yml)
        {
            result.push_back(get<T>(element));
        }
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() << "Error reading yaml sequence of type <" << TYPE_NAME(T) << "> :\n " << e.what());
    }

    return result;
}

template <typename T>
std::set<T> YamlReader::get_set(const Yaml& yml, const std::string& tag)
{
    std::list<T> elements_list = get_list<T>(yml, tag);
    return std::set<T>(elements_list.begin(), elements_list.end());
}

template <typename T>
T YamlReader::get_enumeration(const Yaml& yml, const std::map<std::string, T>& enum_values)
{
    std::string value = get_scalar<std::string>(yml);

    // Find value
    auto it = enum_values.find(value);

    if (it == enum_values.end())
    {
        throw ConfigurationException(
            utils::Formatter() << "Value <" << value << "> is not valid in enumeration <" << TYPE_NAME(T) << ".");
    }
    else
    {
        return it->second;
    }
}

template <typename T>
T YamlReader::get_enumeration(
    const Yaml& yml,
    const std::string& tag,
    const std::map<std::string, T>& enum_values)
{
    return get_enumeration<T>(get_value_in_tag(yml, tag), enum_values);
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_YAML_YAMLREADER_IPP_ */
