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

#ifndef _DDSROUTER_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_

#include <set>
#include <memory>

#include <ddsrouter/configuration/BaseConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
class DDSRouterConfiguration : public BaseConfiguration
{
public:

    /**
     * TODO
     */
    DDSRouterConfiguration(
            std::set<std::shared_ptr<FilterTopic>> allowlist,
            std::set<std::shared_ptr<FilterTopic>> blocklist,
            std::set<std::shared_ptr<RealTopic>> builtin_topics,
            std::set<std::shared_ptr<ParticipantConfiguration>> participants_configurations);

    /**
     * @brief Return a set with the topics allowed in the configuration
     *
     * @return List of filters to get allowed topics
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     */
    std::set<std::shared_ptr<FilterTopic>> allowlist() const noexcept;

    /**
     * @brief Return a set with the topics blocked in the configuration
     *
     * @return List of filters to get blocked topics
     *
     * @throw \c ConfigurationException in case the yaml inside blocklist is not well-formed
     */
    std::set<std::shared_ptr<FilterTopic>> blocklist() const noexcept;

    /**
     * TODO
     */
    std::set<std::shared_ptr<RealTopic>> builtin_topics() const noexcept;

    /**
     * @brief Return a set with the different \c ParticipantConfigurations in the yaml
     *
     * Every tag inside the yaml that is not a key word for the DDSRouterConfiguration could be a Participant.
     * This tag is taken as the \c ParticipantId of this Participant, and a new \c ParticipantConfiguration
     * is created and added to the set to be returned.
     * In case a non valid configuration is found, an invalid \c ParticipantConfiguration (configuration with
     * invalid \c ParticipantType ) will be added to the set.
     *
     * @return List of \c ParticipantConfigurations
     *
     * @throw \c ConfigurationException in case a Participant is not well configured (e.g. No type)
     */
    std::set<std::shared_ptr<ParticipantConfiguration>> participants_configurations() const noexcept;

    bool is_valid() const noexcept override;

protected:

    std::set<std::shared_ptr<FilterTopic>> allowlist_;
    std::set<std::shared_ptr<FilterTopic>> blocklist_;
    std::set<std::shared_ptr<RealTopic>> builtin_topics_;
    std::set<std::shared_ptr<ParticipantConfiguration>> participants_configurations_;
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DDSROUTERCONFIGURATION_HPP_ */
