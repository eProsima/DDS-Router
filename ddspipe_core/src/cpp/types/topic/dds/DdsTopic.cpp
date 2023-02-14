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

#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>
#include <ddspipe_core/types/data/RtpsPayloadData.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

DdsTopic::DdsTopic()
{
    m_internal_type_discriminator = INTERNAL_TOPIC_TYPE_RTPS;
}

std::ostream& operator <<(
        std::ostream& os,
        const DdsTopic& t)
{
    os << "DdsTopic{" << t.topic_name() << ";" << t.type_name << t.topic_qos << "}";
    return os;
}

/////////////////////////
// METHODS
/////////////////////////

bool DdsTopic::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (!Topic::is_valid(error_msg))
    {
        return false;
    }

    return is_valid_dds_topic(m_topic_name, type_name, error_msg);
}

std::string DdsTopic::topic_unique_name() const noexcept
{
    return Topic::topic_unique_name() + type_name;
}

/////////////////////////
// STATIC METHODS
/////////////////////////

bool DdsTopic::is_valid_dds_topic(
        const std::string& topic_name,
        const std::string& type_name,
        utils::Formatter& error_msg) noexcept
{
    if (topic_name.empty())
    {
        error_msg << "Topic name could not be empty. ";
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

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
