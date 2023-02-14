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
 * @file RpcTopic.cpp
 *
 */

#include <regex>

#include <cpp_utils/utils.hpp>

#include <ddspipe_core/types/topic/rpc/RpcTopic.hpp>
#include <ddspipe_core/types/data/RpcPayloadData.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

const std::string RpcTopic::REQUEST_PREFIX_STR = "rq/";
const std::string RpcTopic::REPLY_PREFIX_STR = "rr/";
const std::string RpcTopic::REQUEST_STR = "Request";
const std::string RpcTopic::REPLY_STR = "Reply";
const std::string RpcTopic::RESPONSE_STR = "Response";

RpcTopic::RpcTopic(
        const std::string& service_name,
        const DdsTopic& request_topic,
        const DdsTopic& reply_topic) noexcept
    : service_name_(service_name)
    , request_topic_(request_topic)
    , reply_topic_(reply_topic)
{
}

RpcTopic::RpcTopic(
        const DdsTopic& topic) noexcept
{
    if (is_request_topic(topic))
    {
        request_topic_ = topic;

        reply_topic_ = topic;
        reply_topic_.m_topic_name =
                std::regex_replace(reply_topic_.m_topic_name, std::regex(REQUEST_PREFIX_STR), REPLY_PREFIX_STR);
        reply_topic_.m_topic_name =
                std::regex_replace(reply_topic_.m_topic_name, std::regex(REQUEST_STR), REPLY_STR);
        reply_topic_.type_name =
                std::regex_replace(reply_topic_.type_name, std::regex(REQUEST_STR), RESPONSE_STR);

        service_name_ = std::regex_replace(topic.m_topic_name, std::regex(REQUEST_PREFIX_STR + "|" + REQUEST_STR), "");
    }
    else if (is_reply_topic(topic))
    {
        reply_topic_ = topic;

        request_topic_ = topic;
        request_topic_.m_topic_name =
                std::regex_replace(request_topic_.m_topic_name, std::regex(REPLY_PREFIX_STR), REQUEST_PREFIX_STR);
        request_topic_.m_topic_name =
                std::regex_replace(request_topic_.m_topic_name, std::regex(REPLY_STR), REQUEST_STR);
        request_topic_.type_name =
                std::regex_replace(request_topic_.type_name, std::regex(RESPONSE_STR), REQUEST_STR);

        service_name_ = std::regex_replace(topic.m_topic_name, std::regex(REPLY_PREFIX_STR + "|" + REPLY_STR), "");
    }
    else
    {
        utils::tsnh(
            utils::Formatter() << "Attempting to create RpcTopic from invalid topic.");
    }

    reply_topic_.m_internal_type_discriminator = INTERNAL_TOPIC_TYPE_RPC;
    request_topic_.m_internal_type_discriminator = INTERNAL_TOPIC_TYPE_RPC;

    // Set both topic qos as the one found
    request_topic_.topic_qos = topic.topic_qos;
    reply_topic_.topic_qos = topic.topic_qos;
}

RpcTopic::RpcTopic(const RpcTopic& other) noexcept
    : service_name_(other.service_name_)
    , request_topic_(other.request_topic_)
    , reply_topic_(other.reply_topic_)
{

}

const std::string& RpcTopic::service_name() const
{
    return service_name_;
}

const DdsTopic& RpcTopic::request_topic() const
{
    return request_topic_;
}

const DdsTopic& RpcTopic::reply_topic() const
{
    return reply_topic_;
}

bool RpcTopic::is_request_topic(
        const DdsTopic& topic)
{
    std::string m_topic_name = topic.m_topic_name;
    std::string type_name = topic.type_name;

    return (m_topic_name.find(REQUEST_PREFIX_STR) == 0) &&
           (m_topic_name.rfind(REQUEST_STR) + REQUEST_STR.length() == m_topic_name.length()) &&
           (type_name.rfind(REQUEST_STR) + REQUEST_STR.length() + 1 == type_name.length());
}

bool RpcTopic::is_reply_topic(
        const DdsTopic& topic)
{
    std::string m_topic_name = topic.m_topic_name;
    std::string type_name = topic.type_name;

    return (m_topic_name.find(REPLY_PREFIX_STR) == 0) &&
           (m_topic_name.rfind(REPLY_STR) + REPLY_STR.length() == m_topic_name.length()) &&
           (type_name.rfind(RESPONSE_STR) + RESPONSE_STR.length() + 1 == type_name.length());
}

bool RpcTopic::is_service_topic(
        const DdsTopic& topic)
{
    return is_request_topic(topic) || is_reply_topic(topic);
}

bool RpcTopic::operator <(
        const RpcTopic& other) const
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
        const RpcTopic& a)
{
    os << "RpcTopic{" << a.service_name() << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
