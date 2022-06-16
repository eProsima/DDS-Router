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
 * @file DDSRouterReloadConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_

#include <memory>
#include <set>

#include <ddsrouter_utils/Formatter.hpp>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
class DDSRouterReloadConfiguration
{
public:

    /**
     * @brief Constructor from a set of allowed, blocked and builtin topics
     *
     * @param allowlist Allowed topics
     * @param blocklist Blocked topics
     * @param builtin_topics Builtin topics
     *
     */
    DDSROUTER_CORE_DllAPI DDSRouterReloadConfiguration(
            types::TopicKeySet<types::FilterTopic> allowlist,
            types::TopicKeySet<types::FilterTopic> blocklist,
            types::TopicKeySet<types::RealTopic> builtin_topics);

    /**
     * @brief Return a set with the topics allowed in the configuration
     *
     * @return Const reference of allowed topics
     */
    DDSROUTER_CORE_DllAPI const types::TopicKeySet<types::FilterTopic>& allowlist() const noexcept;

    /**
     * @brief Return a set with the topics blocked in the configuration
     *
     * @return Const reference of blocked topics
     */
    DDSROUTER_CORE_DllAPI const types::TopicKeySet<types::FilterTopic>& blocklist() const noexcept;

    /**
     *@brief Return a set of all registered topics
     *
     * @return Const reference of builtin topics
     */
    DDSROUTER_CORE_DllAPI const types::TopicKeySet<types::RealTopic>& builtin_topics() const noexcept;

    /**
     * @brief Register a new topic
     *
     * @return Set of filters to get blocked topics
     */
    DDSROUTER_CORE_DllAPI bool register_topic(
            const types::RealTopic& topic);

    /**
     * @brief Check topic registration
     *
     * @return whether a topic has been registered.
     */
    DDSROUTER_CORE_DllAPI bool is_topic_registered(
            const types::RealTopic& topic) const noexcept;

    /**
     * @brief Overwrite topics sets from an external configuration
     *
     */
    DDSROUTER_CORE_DllAPI types::TopicKeySet<types::RealTopic> reload(
            const DDSRouterReloadConfiguration& new_configuration);

    /**
     * @brief Return whether a topic is allowed.
     *
     */
    DDSROUTER_CORE_DllAPI bool is_topic_allowed(
            const types::RealTopic& topic) const noexcept;


    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const DDSRouterReloadConfiguration& cfg);

protected:

    //! Owned allow list of Filter Topics
    types::TopicKeySet<types::FilterTopic> allowlist_;

    //! Owned block list of Filter Topics
    types::TopicKeySet<types::FilterTopic> blocklist_;

    //! Owned list of Real Topics
    types::TopicKeySet<types::RealTopic> builtin_topics_;
};

//! \c AllowedTopicList to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const DDSRouterReloadConfiguration& cfg);


} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_ */
