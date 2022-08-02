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

struct RPCTopic
{
    RPCTopic(
            const std::string& service_name,
            const RealTopic& topic_request,
            const RealTopic& topic_reply) noexcept;

    RPCTopic(
            const RealTopic& topic) noexcept;

    const std::string& service_name() const;

    const RealTopic& request_topic() const;

    const RealTopic& reply_topic() const;

    static bool is_request_topic(const RealTopic& topic);

    static bool is_reply_topic(const RealTopic& topic);

    static bool is_service_topic(const RealTopic& topic);

    bool operator <(
            const RPCTopic& other) const;

protected:

    std::string service_name_;

    RealTopic request_topic_;

    RealTopic reply_topic_;

    static const std::string request_prefix_str;
    static const std::string reply_prefix_str;
    static const std::string request_str;
    static const std::string reply_str;
    static const std::string response_str;
};

/**
 * Serialization method
 *
 * It prints the topic name, type, kind and reliability inside "{}" and separated by ";"
 * Example: {TopicName;TopicType;no_key;reliable}
 */
std::ostream& operator <<(
        std::ostream& os,
        const RPCTopic& a);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_RPCTOPIC_HPP_ */
