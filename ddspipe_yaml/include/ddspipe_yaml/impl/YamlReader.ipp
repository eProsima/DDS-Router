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

#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/macros/macros.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/types/Fuzzy.hpp>

namespace eprosima {
namespace ddspipe {
namespace yaml {

template <typename T>
T YamlReader::get(
        const Yaml& yml,
        const TagType& tag,
        const YamlReaderVersion version)
{
    // ATTENTION: This try catch can be avoided, it is only used to add verbose information
    try
    {
        return get<T>(get_value_in_tag(yml, tag), version);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() <<
                      "Error getting required value of type <" << TYPE_NAME(T) <<
                      "> in tag <" << tag << "> :\n " << e.what());
    }
}

template <typename T>
void YamlReader::fill(
        T& object,
        const Yaml& yml,
        const TagType& tag,
        const YamlReaderVersion version)
{
    // ATTENTION: This try catch can be avoided, it is only used to add verbose information
    try
    {
        return fill<T>(get_value_in_tag(yml, tag), object, version);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() <<
                      "Error filling object of type <" << TYPE_NAME(T) <<
                      "> in tag <" << tag << "> :\n " << e.what());
    }
}

template <typename T>
T YamlReader::get_scalar(
        const Yaml& yml,
        const TagType& tag)
{
    try
    {
        return get_scalar<T>(get_value_in_tag(yml, tag));
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error reading yaml scalar under tag <" << tag << "> :\n " << e.what());
    }
}

template <typename T>
T YamlReader::get_scalar(
        const Yaml& yml)
{
    if (!yml.IsScalar())
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() <<
                      "Trying to read a primitive value of type <" << TYPE_NAME(T) << "> from a non scalar yaml.");
    }

    try
    {
        return yml.as<T>();
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() <<
                      "Incorrect format for primitive value, expected <" << TYPE_NAME(T) << ">:\n " << e.what());
    }
}

template <typename T>
std::list<T> YamlReader::get_list(
        const Yaml& yml,
        const TagType& tag,
        const YamlReaderVersion version)
{
    try
    {
        return get_list<T>(get_value_in_tag(yml, tag), version);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error reading yaml list under tag <" << tag << "> :\n " << e.what());
    }
}

template <typename T>
std::list<T> YamlReader::get_list(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    if (!yml.IsSequence())
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Incorrect format, yaml Sequence expected.");
    }

    std::list<T> result;

    try
    {
        for (Yaml element : yml)
        {
            result.push_back(get<T>(element, version));
        }
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error reading yaml sequence of type <" << TYPE_NAME(T) << "> :\n " <<
                      e.what());
    }

    return result;
}

template <typename T>
std::set<T> YamlReader::get_set(
        const Yaml& yml,
        const TagType& tag,
        const YamlReaderVersion version)
{
    std::list<T> elements_list = get_list<T>(yml, tag, version);
    return std::set<T>(elements_list.begin(), elements_list.end());
}

template <typename T>
T YamlReader::get_enumeration(
        const Yaml& yml,
        const TagType& tag,
        const std::map<TagType, T>& enum_values)
{
    try
    {
        return get_enumeration<T>(get_value_in_tag(yml, tag), enum_values);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Error reading enumeration value under tag <" << tag << "> :\n " << e.what());
    }
}

template <typename T>
T YamlReader::get_enumeration(
        const Yaml& yml,
        const std::map<TagType, T>& enum_values)
{
    TagType value = get_scalar<TagType>(yml);

    // Find value
    auto it = enum_values.find(value);

    if (it == enum_values.end())
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Value <" << value << "> is not valid in enumeration <" << TYPE_NAME(T) << ".");
    }
    else
    {
        return it->second;
    }
}

template <typename T>
T YamlReader::get_enumeration_from_builder(
        const Yaml& yml,
        const utils::EnumBuilder<T>& enum_builder)
{
    std::string str_value = get_scalar<std::string>(yml);
    T enum_value;
    bool found = enum_builder.string_to_enumeration(str_value, enum_value);

    if (found)
    {
        return enum_value;
    }
    else
    {
        throw eprosima::utils::ConfigurationException(utils::Formatter()
            << "Value <" << str_value << "> cannot be parsed as enum <" << TYPE_NAME(T) << ">.");
    }
}

template <typename T>
T YamlReader::get_enumeration_from_builder(
        const Yaml& yml,
        const TagType& tag,
        const utils::EnumBuilder<T>& enum_builder)
{
    try
    {
        return get_enumeration_from_builder<T>(get_value_in_tag(yml, tag), enum_builder);
    }
    catch (const std::exception& e)
    {
        throw eprosima::utils::ConfigurationException(utils::Formatter()
            << "Error reading enumeration from builder under tag <"
            << tag << "> :\n " << e.what());
    }
}

} /* namespace yaml */
} /* namespace ddspipe */
} /* namespace eprosima */
