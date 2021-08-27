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
 * @file Address.cpp
 *
 */

#include <fastdds/dds/log/Log.hpp>

#include <databroker/Address.hpp>
#include <databroker/utils.hpp>

namespace eprosima {
namespace databroker {

Address::Address(
        const std::string& ip,
        const uint32_t port,
        const eprosima::fastrtps::rtps::GuidPrefix_t& guid)
    : ip(ip)
    , port(port)
    , guid(guid)
{
}

Address::Address(
        const std::string& ip,
        const uint32_t port,
        const uint32_t id)
    : Address(ip, port, guid_server(id))
{
}

Address::Address(
        const std::string& ip,
        const uint32_t port,
        const std::string& guid)
    : Address(ip, port, guid_server(guid))
{
}

// TODO in general, check gramar allowed in string input before conversions
Address::Address(
        const std::string& address)
    : Address()
{
    std::vector<std::string> fields;
    utils::split_string(address, fields, ",");

    if (fields.size() == 2)
    {
        // If there are two fields, the server id is skipped
        ip = fields[0];
        port = std::stol(fields[1]);
    }
    else if (fields.size() == 3)
    {
        // TODO allow server guid as well
        ip = fields[0];
        port = std::stol(fields[1]);
        guid = guid_server(std::stol(fields[2]));
    }
    else
    {
        logError(DATABROKER_ADDRESS, "ERROR Incorrect address format " << address);
    }
}

Address::Address()
    : Address("127.0.0.1", 11800, guid_server(0))
{
}

Address::~Address()
{
}

bool Address::read_addresses_vector(
        const std::string& addresses,
        std::vector<Address>& result)
{
    std::vector<std::string> addresses_split;

    if (!utils::split_string(addresses, addresses_split))
    {
        return false;
    }

    for (auto add : addresses_split)
    {
        result.push_back(Address(add));
    }

    return true;
}

Address Address::read_address(
        const std::string& address)
{
    return Address(address);
}

bool Address::correct_ip(
        const std::string& ip)
{
    // TODO
    return false;
}

eprosima::fastrtps::rtps::GuidPrefix_t Address::guid_server()
{
    return guid_server(0);
}

eprosima::fastrtps::rtps::GuidPrefix_t Address::guid_server(
        uint8_t id)
{
    eprosima::fastrtps::rtps::GuidPrefix_t guid;
    std::istringstream(SERVER_DEFAULT_GUID) >> guid;
    guid.value[SERVER_DEFAULT_GUID_ID_INDEX] = static_cast<unsigned char>(id);
    return guid;
}

eprosima::fastrtps::rtps::GuidPrefix_t Address::guid_server(
        const std::string& server_guid)
{
    eprosima::fastrtps::rtps::GuidPrefix_t guid;
    std::istringstream(server_guid) >> guid;
    return guid; // There is no easy wat to directly return the guid
}

std::string Address::guid_to_string(
        const eprosima::fastrtps::rtps::GuidPrefix_t& guid)
{
    std::ostringstream guid_ostream;
    guid_ostream << guid;
    return guid_ostream.str();
}

} /* namespace databroker */
} /* namespace eprosima */
