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
 * @file utils.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_UTILS_HPP
#define EPROSIMA_DATABROKER_UTILS_HPP

#include <string>

#include <fastdds/rtps/common/GuidPrefix_t.hpp>

#include <yaml-cpp/yaml.h>

namespace eprosima {
namespace databroker {
namespace utils {

#define SERVER_DEFAULT_GUID "01.0f.00.44.41.54.95.42.52.4f.4b.45.52"
#define SERVER_DEFAULT_GUID_ID_INDEX 2

inline eprosima::fastrtps::rtps::GuidPrefix_t guid_server(
        uint8_t id)
{
    eprosima::fastrtps::rtps::GuidPrefix_t guid;
    std::istringstream(SERVER_DEFAULT_GUID) >> guid;
    guid.value[SERVER_DEFAULT_GUID_ID_INDEX] = static_cast<unsigned char>(id);
    return guid;
}

inline eprosima::fastrtps::rtps::GuidPrefix_t guid_server(
        const YAML::Node& server_id,
        const YAML::Node& server_guid)
{
    if (server_guid)
    {
        // Server GUID set and used
        eprosima::fastrtps::rtps::GuidPrefix_t guid;
        std::istringstream(server_guid.as<std::string>()) >> guid;
        return guid; // There is no easy wat to directly return the guid
    }
    else if (server_id)
    {
        // Server ID set without GUID set
        return guid_server(server_id.as<uint32_t>() % std::numeric_limits<uint8_t>::max());
    }
    else
    {
        // Server GUID by default with ID 0
        return guid_server(0);
    }
}

inline std::string guid_to_string(
    const eprosima::fastrtps::rtps::GuidPrefix_t& guid)
{
    std::ostringstream guid_ostream;
    guid_ostream << guid;
    return guid_ostream.str();
}

} /* namespace utils */
} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_UTILS_HPP */
