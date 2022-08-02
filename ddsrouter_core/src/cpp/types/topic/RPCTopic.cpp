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
 * @file RPCTopic.cpp
 *
 */

#include <regex>

#include <ddsrouter_core/types/topic/RPCTopic.hpp>
// #include <ddsrouter_utils/exception/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

const std::string RPCTopic::request_prefix_str = "rq/";
const std::string RPCTopic::reply_prefix_str = "rr/";
const std::string RPCTopic::request_str = "Request";
const std::string RPCTopic::reply_str = "Reply";
const std::string RPCTopic::response_str = "Response";

RPCTopic::RPCTopic(
        const std::string& service_name,
        const RealTopic& request_topic,
        const RealTopic& reply_topic) noexcept
    : service_name_(service_name)
    , request_topic_(request_topic)
    , reply_topic_(reply_topic)
{
}

RPCTopic::RPCTopic(
        const RealTopic& topic) noexcept
{
    if (is_request_topic(topic))
    {
        request_topic_ = topic;

        reply_topic_ = topic;
        reply_topic_.topic_name(std::regex_replace(reply_topic_.topic_name(), std::regex(request_prefix_str), reply_prefix_str));
        reply_topic_.topic_name(std::regex_replace(reply_topic_.topic_name(), std::regex(request_str), reply_str));
        reply_topic_.topic_type(std::regex_replace(reply_topic_.topic_type(), std::regex(request_str), response_str));

        service_name_ = std::regex_replace(topic.topic_name(), std::regex(request_prefix_str + "|" + request_str), "");
    }
    else if (is_reply_topic(topic))
    {
        reply_topic_ = topic;

        request_topic_ = topic;
        request_topic_.topic_name(std::regex_replace(request_topic_.topic_name(), std::regex(reply_prefix_str), request_prefix_str));
        request_topic_.topic_name(std::regex_replace(request_topic_.topic_name(), std::regex(reply_str), request_str));
        request_topic_.topic_type(std::regex_replace(request_topic_.topic_type(), std::regex(response_str), request_str));

        service_name_ = std::regex_replace(topic.topic_name(), std::regex(reply_prefix_str + "|" + reply_str), "");
    }
    else
    {
        // tsnh or throw exception
        std::cout << "Error" << std::endl;
    }
}

const std::string& RPCTopic::service_name() const
{
    return service_name_;
}

const RealTopic& RPCTopic::request_topic() const
{
    return request_topic_;
}

const RealTopic& RPCTopic::reply_topic() const
{
    return reply_topic_;
}

bool RPCTopic::is_request_topic(const RealTopic& topic)
{
    std::string topic_name = topic.topic_name();
    std::string topic_type = topic.topic_type();

    return (topic_name.find(request_prefix_str) == 0) && (topic_name.rfind(request_str) + request_str.length() == topic_name.length()) && (topic_type.rfind(request_str) + request_str.length() + 1 == topic_type.length());
}

bool RPCTopic::is_reply_topic(const RealTopic& topic)
{
    std::string topic_name = topic.topic_name();
    std::string topic_type = topic.topic_type();

    return (topic_name.find(reply_prefix_str) == 0) && (topic_name.rfind(reply_str) + reply_str.length() == topic_name.length()) && (topic_type.rfind(response_str) + response_str.length() + 1 == topic_type.length());
}

bool RPCTopic::is_service_topic(const RealTopic& topic)
{
    return is_request_topic(topic) || is_reply_topic(topic);
}

bool RPCTopic::operator <(
        const RPCTopic& other) const
{
    int name_comparison = service_name_.compare(other.service_name());
    if (name_comparison < 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const RPCTopic& a)
{
    os << "RPCTopic{" << a.service_name() << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
