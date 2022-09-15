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
 * @file RPCTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_RPCTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_RPCTOPIC_HPP_

#include <ddsrouter_core/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Class that represents a RPC service in a DDS network.
 * It is composed of a pair of DDS topics; one for request and other for reply.
 * These two DDS topics are formed by mangling a \c service_name with different prefix and suffixes.
 */
class RPCTopic
{
public:

    //! Constructor by service name, request topic and reply topic
    RPCTopic(
            const std::string& service_name,
            const RealTopic& topic_request,
            const RealTopic& topic_reply) noexcept;

    //! Constructor by request/reply topic only (infers service name and the other request/reply topic)
    RPCTopic(
            const RealTopic& topic) noexcept;

    //! Service name getter
    const std::string& service_name() const;

    //! Request topic getter
    const RealTopic& request_topic() const;

    //! Reply topic getter
    const RealTopic& reply_topic() const;

    //! Whether a topic is a request topic
    static bool is_request_topic(
            const RealTopic& topic);

    //! Whether a topic is a reply topic
    static bool is_reply_topic(
            const RealTopic& topic);

    //! Whether a topic is a service topic (request or reply topic)
    static bool is_service_topic(
            const RealTopic& topic);

    /**
     * Minor operator
     *
     * It compares both service names lexicographically
     */
    bool operator <(
            const RPCTopic& other) const;

protected:

    //! Name of the service
    std::string service_name_;

    //! Topic used for transmitting requests
    RealTopic request_topic_;

    //! Topic used for transmitting replies
    RealTopic reply_topic_;

    static const std::string REQUEST_PREFIX_STR;
    static const std::string REPLY_PREFIX_STR;
    static const std::string REQUEST_STR;
    static const std::string REPLY_STR;
    static const std::string RESPONSE_STR;
};

/**
 * Serialization method
 *
 * It prints the service name inside "{}"
 * Example: RPCTopic{ServiceName}
 */
std::ostream& operator <<(
        std::ostream& os,
        const RPCTopic& a);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_RPCTOPIC_HPP_ */
