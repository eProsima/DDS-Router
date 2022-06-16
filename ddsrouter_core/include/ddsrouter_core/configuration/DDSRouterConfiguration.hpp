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
 * @file DDSRouterConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_

#include <memory>
#include <set>
#include <vector>

#include <ddsrouter_utils/Formatter.hpp>

#include <ddsrouter_core/configuration/payload_pool/PoolConfig.h>
#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/Topic.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using PayloadPoolIndexHasher = std::hash<std::string>;

using PoolConfigT = fastrtps::rtps::recycle::PoolConfig;

constexpr unsigned int MAX_THREADS = 1024;
constexpr unsigned int DEFAULT_THREADS = 4;

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
class DDSRouterConfiguration : public DDSRouterReloadConfiguration
{
public:

    /**
     * Constructor from a set of allowed, blocked and builtin topics
     *
     * @param allowlist Allowed topics, forwarded to DDSRouterReloadConfiguration
     * @param blocklist Blocked topics, forwarded to DDSRouterReloadConfiguration
     * @param builtin_topics Builtin topics, forwarded to DDSRouterReloadConfiguration
     *
     * @throw ConfigurationException if there is an error with the input parameters
     */
    DDSROUTER_CORE_DllAPI DDSRouterConfiguration(
            types::TopicKeySet<types::FilterTopic> allowlist,
            types::TopicKeySet<types::FilterTopic> blocklist,
            types::TopicKeySet<types::RealTopic> builtin_topics,
            types::ParticipantKeySet<std::shared_ptr<ParticipantConfiguration>> participants_configurations,
            unsigned int threads = DEFAULT_THREADS,
            unsigned int payload_pool_granularity = fastrtps::rtps::recycle::DEFAULT_GRANULARITY,
            PoolConfigT payload_pool_config = PoolConfigT());

    /**
     * @brief Return a unique set with the different \c ParticipantConfigurations in the yaml
     *
     * Every participant configuration is an object of the specific class set in \c types::ParticipantKind .
     *
     * @return Set of not owned const \c ParticipantConfigurations instances
     */
    DDSROUTER_CORE_DllAPI types::ParticipantKeySet<const ParticipantConfiguration*> participants_configurations() const;

    /**
     * @brief Return the number of threads
     *
     */
    DDSROUTER_CORE_DllAPI unsigned int threads() const noexcept;

    /**
     * @brief Return the payload pool granularity
     *
     */
    DDSROUTER_CORE_DllAPI unsigned int payload_pool_granularity() const noexcept;

    /**
     * @brief Return a const reference to the pool configuration
     *
     */
    DDSROUTER_CORE_DllAPI const PoolConfigT& payload_pool_configuration() const noexcept;

    /**
     * @brief Return a string index used to access a payload pool object
     *
     */
    DDSROUTER_CORE_DllAPI std::string get_payload_pool_index(
            const std::string& original_index) const noexcept;

    DDSROUTER_CORE_DllAPI unsigned int number_of_threads() const noexcept;

    DDSROUTER_CORE_DllAPI static unsigned int default_number_of_threads() noexcept;

protected:

    //! Owning set of participants configurations
    types::ParticipantKeySet<std::shared_ptr<ParticipantConfiguration>> participants_configurations_;

    //! Number of threads in the thread pool
    unsigned int threads_;

    //! Payload pool granularity
    unsigned int payload_pool_granularity_;

    //! Payload pool configuration
    PoolConfigT payload_pool_config_;

    //! Hasher functor to map topic to its associated payload pool
    PayloadPoolIndexHasher payload_pool_index_hasher_;

private:

    //! Internal throwing validator
    void check_valid_() const;
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_ */
