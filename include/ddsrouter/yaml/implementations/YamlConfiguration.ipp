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
 * @file YamlManager.cpp
 *
 */

#include <ddsrouter/exception/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

template <typename T>
T YamlConfiguration::get(const Yaml& yml, std::string tag)
{
    return get<T>(get_value_in_tag(yml, tag));
}

template <typename T>
T YamlConfiguration::get_native(const Yaml& yml, std::string tag)
{
    return get_native<T>(get_value_in_tag(yml, tag));
}

template <typename T>
T YamlConfiguration::get_native(const Yaml& yml)
{
    try
    {
        return yml.as<T>();
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(
            utils::Formatter() << "Incorrect format for primitive value. Erro: " << e.what());
    }
}

template <typename T>
std::list<T> YamlConfiguration::get_list(const Yaml& yml, std::string tag)
{
    return get_list<T>(get_value_in_tag(yml, tag));
}

template <typename T>
std::list<T> YamlConfiguration::get_list(const Yaml& yml)
{
    if (!yml.IsSequence())
    {
        // TODO error
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
        // TODO error
    }

    return result;
}

template <typename T>
std::set<T> YamlConfiguration::get_set(const Yaml& yml, std::string tag)
{
    std::list<T> elements_list = get_list<T>(yml, tag);
    return std::set<T>(elements_list.begin(), elements_list.end());
}

template <typename T>
T YamlConfiguration::get_enumeration(const Yaml& yml, std::map<std::string, T>enum_values)
{
    std::string value = get_native<std::string>(yml);

    // Find value
    auto it = enum_values.find(value);

    if (it == enum_values.end())
    {
        throw ConfigurationException(
            utils::Formatter() << "Enumeration Value: " << value << " is not a valid value.");
    }
    else
    {
        return it->second;
    }
}

template <typename T>
T YamlConfiguration::get_enumeration(const Yaml& yml, std::string tag, std::map<std::string, T>enum_values)
{
    return get_enumeration<T>(get_value_in_tag(yml, tag), enum_values);
}

template <typename T>
bool YamlConfiguration::is_present(const Yaml& yml)
{
    try
    {
        get<T>(yml);
    }
    catch(const std::exception& e)
    {
        return false;
    }

    return true;
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
