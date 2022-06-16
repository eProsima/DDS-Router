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
 * @file DDSRouterConfiguration.cpp
 *
 */

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.ipp>
#include <ddsrouter_core/types/topic/Topic.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using namespace eprosima::ddsrouter::core::types;


DDSRouterConfiguration::DDSRouterConfiguration(
        TopicKeySet<FilterTopic> allowlist,
        TopicKeySet<FilterTopic> blocklist,
        TopicKeySet<RealTopic> builtin_topics,
        ParticipantKeySet<std::shared_ptr<ParticipantConfiguration>> participants_configurations,
        unsigned int threads,
        unsigned int payload_pool_granularity,
        PoolConfigT payload_pool_config)
    : DDSRouterReloadConfiguration (std::move(allowlist), std::move(blocklist), std::move(builtin_topics))
    , participants_configurations_(std::move(participants_configurations))
    , threads_(threads)
    , payload_pool_granularity_(payload_pool_granularity)
    , payload_pool_config_(payload_pool_config)
{
    this->check_valid_();
}

types::ParticipantKeySet<const ParticipantConfiguration*> DDSRouterConfiguration::participants_configurations() const
{
    types::ParticipantKeySet<const ParticipantConfiguration*> raw_participants_configurations;

    for (const auto& configuration : participants_configurations_)
    {
        raw_participants_configurations.insert(configuration.get());
    }

    return raw_participants_configurations;
}

DDSROUTER_CORE_DllAPI void DDSRouterConfiguration::check_valid_() const
{
    // Check there are at least two participants
    if (participants_configurations_.size() < 2)
    {
        throw utils::ConfigurationException( utils::Formatter() << "There must be at least 2 participants. ");
    }

    if (threads_ > MAX_THREADS)
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Maximum number of threads " << MAX_THREADS << " exceeded: " <<
                      threads_);
    }

    if (threads_ == 0)
    {
        throw utils::ConfigurationException(utils::Formatter() << "Expected non-zero threads");
    }
}

DDSROUTER_CORE_DllAPI unsigned int DDSRouterConfiguration::threads() const noexcept
{
    return threads_;
}

DDSROUTER_CORE_DllAPI unsigned int DDSRouterConfiguration::payload_pool_granularity() const noexcept
{
    return payload_pool_granularity_;
}

DDSROUTER_CORE_DllAPI const fastrtps::rtps::recycle::PoolConfig& DDSRouterConfiguration::payload_pool_configuration()
const noexcept
{
    return payload_pool_config_;
}

DDSROUTER_CORE_DllAPI std::string DDSRouterConfiguration::get_payload_pool_index(
        const std::string& original_index) const noexcept
{
    if (payload_pool_granularity_ == 0)
    {
        return original_index;
    }
    else
    {
        return std::to_string(payload_pool_index_hasher_(original_index) % payload_pool_granularity_) + "_h";
    }
}

unsigned int DDSRouterConfiguration::number_of_threads() const noexcept
{
    return number_of_threads_;
}

unsigned int DDSRouterConfiguration::default_number_of_threads() noexcept
{
    return DEFAULT_NUMBER_OF_THREADS_;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
