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
 * @file DdsTopic.cpp
 *
 */

#include <string>
#include <vector>

#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>
#include <cpp_utils/exception/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

DdsTopic::DdsTopic(
        const std::string& topic_name,
        const std::string& type_name) noexcept
    : Topic(topic_name)
    , type_name(type_name)
{
}

DdsTopic::DdsTopic(
        const std::string& topic_name,
        const std::string& type_name,
        const bool keyed,
        const types::TopicQoS& qos) noexcept
    : Topic(topic_name)
    , type_name(type_name)
    , keyed(keyed)
    , topic_qos(qos)
{
}

bool DdsTopic::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (!Topic::is_valid(error_msg))
    {
        return false;
    }

    if (type_name.empty())
    {
        error_msg << "Topic type name could not be empty. ";
        return false;
    }

    // TODO: extend with regex
    // It checks if topic name or type contain an invalid substring
    std::vector<std::string> invalid_substrings = {
        "*", // Wildcard char
    };

    for (std::string invalid_substring : invalid_substrings)
    {
        if (topic_name.find(invalid_substring) != std::string::npos)
        {
            error_msg << "Topic name could not contain " << invalid_substring << " . ";
            return false;
        }
        else if (type_name.find(invalid_substring) != std::string::npos)
        {
            error_msg << "Topic type name could not contain " << invalid_substring << " . ";
            return false;
        }
    }

    return true;
}

bool DdsTopic::operator < (
        const DdsTopic& other) const noexcept
{
    if (Topic::operator ==(other))
    {
        return this->type_name < other.type_name;
    }
    return Topic::operator <(other);
}

bool DdsTopic::operator == (
        const DdsTopic& other) const noexcept
{
    return
        Topic::operator ==(other) &&
        this->type_name == other.type_name;
}

bool DdsTopic::is_valid_dds_topic(
        const std::string& topic_name,
        const std::string& type_name) noexcept
{
    utils::Formatter __f;
    return DdsTopic(topic_name, type_name).is_valid(__f);
}

std::ostream& operator <<(
        std::ostream& os,
        const DdsTopic& t)
{
    std::string keyed_str = t.keyed ? ";keyed;" : ";";

    os << "DdsTopic{" << t.topic_name << ";" << t.type_name << keyed_str << t.topic_qos << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
