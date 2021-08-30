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
 * @file Address.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_ADDRESS_HPP
#define EPROSIMA_DATABROKER_ADDRESS_HPP

#include <string>
#include <vector>

#include <fastdds/rtps/common/GuidPrefix_t.hpp>

namespace eprosima {
namespace databroker {

#define SERVER_DEFAULT_GUID "01.0f.00.44.41.54.41.5f.42.52.4f.4b"
#define SERVER_DEFAULT_GUID_ID_INDEX 2

struct Address
{
    Address(
            const std::string& ip,
            const uint32_t port,
            const eprosima::fastrtps::rtps::GuidPrefix_t& guid);

    Address(
            const std::string& ip,
            const uint32_t port,
            const uint32_t id);

    Address(
            const std::string& ip,
            const uint32_t port,
            const std::string& guid);

    Address(
            const std::string& address);
    Address();

    virtual ~Address();

    static bool read_addresses_vector(
            const std::string& addresses,
            std::vector<Address>& result);

    static Address read_address(
            const std::string& address);

    static bool correct_ip(
            const std::string& ip);

    static eprosima::fastrtps::rtps::GuidPrefix_t guid_server();

    static eprosima::fastrtps::rtps::GuidPrefix_t guid_server(
            uint8_t id);

    static eprosima::fastrtps::rtps::GuidPrefix_t guid_server(
            const std::string& server_guid);

    static std::string guid_to_string(
            const eprosima::fastrtps::rtps::GuidPrefix_t& guid);

    std::string ip;
    uint32_t port;
    eprosima::fastrtps::rtps::GuidPrefix_t guid;
};

inline std::ostream& operator <<(
        std::ostream& output,
        const Address& ad)
{
    output << ad.ip << "," << ad.port << "," << ad.guid;
    return output;
}

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_ADDRESS_HPP */
