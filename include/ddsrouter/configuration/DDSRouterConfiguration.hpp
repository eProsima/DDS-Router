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

#ifndef _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/BaseConfiguration.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
class DDSRouterConfiguration : public BaseConfiguration
{
public:

    /**
     * @brief Construct a new DDSRouterConfiguration object from a yaml object
     *
     * @param raw_configuration represents the yaml to this object to get the configuration data. This yaml must be a
     *  map or an empty yaml, and nothing else
     *
     * @throw \c ConfigurationException in case the yaml is not well formed or is not a map nor empty
     */
    DDSRouterConfiguration(
            const RawConfiguration& raw_configuration);

    /**
     * @brief Return a list with the topics allowed in the configuration
     *
     * @return List of filters to get allowed topics
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     */
    std::list<std::shared_ptr<FilterTopic>> allowlist() const;

    /**
     * @brief Return a list with the topics blocked in the configuration
     *
     * @return List of filters to get blocked topics
     *
     * @throw \c ConfigurationException in case the yaml inside blocklist is not well-formed
     */
    std::list<std::shared_ptr<FilterTopic>> blocklist() const;

    /**
     * @brief Return a list with the different \c ParticipantConfigurations in the yaml
     *
     * Every tag inside the yaml that is not a key word for the DDSRouterConfiguration could be a Participant.
     * This tag is taken as the \c ParticipantId of this Participant, and a new \c ParticipantConfiguration
     * is created and added to the list to be returned.
     * In case a non valid configuration is found, an invalid \c ParticipantConfiguration (configuration with
     * invalid \c ParticipantType ) will be added to the list.
     *
     * @return List of \c ParticipantConfigurations
     */
    std::list<ParticipantConfiguration> participants_configurations() const noexcept;


    /**
     * @brief Return a set with the real topics included in allowedlist
     *
     * This method gets the allowedlist in configuration and returns only those topics that can actually
     * be created as Real topics.
     *
     * @todo: This method will disappear once the dynamic module is implemented
     *
     * @return Set of real topics
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     */
    std::set<RealTopic> real_topics() const;

protected:

    /**
     * @brief Generic method to get a list of topics from a list
     *
     * This method collects the same functionality that share \c allowlist and \c blocklist
     *
     * @param [in] list_tag: tag name of the list to look for in the configuration
     * @return List of filter topics
     */
    std::list<std::shared_ptr<FilterTopic>> generic_get_topic_list_(
            const char* list_tag) const;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_ */
