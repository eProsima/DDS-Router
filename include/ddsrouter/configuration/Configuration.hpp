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
 * @file Configuration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This class joins every DDSRouter Configuration characteristic and give methods to interact with this configuration.
 */
class Configuration
{
public:

    /**
     * @brief Construct a new Configuration object from a yaml object
     *
     * @param raw_configuration represents the yaml to this object to get the configuration data. This yaml must be a
     *  map or an empty yaml, it cannot be otherwise.
     *
     * @throw \c ConfigurationException in case the yaml is not well formed or is not a map or empty
     */
    Configuration(
            const RawConfiguration& raw_configuration);

    /**
     * @brief Return a list with topics in allowedlist in the configuration
     *
     * @return List of filters to get allowed topics
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is bad formed
     */
    std::list<std::shared_ptr<FilterTopic>> allowlist() const;

    /**
     * @brief Return a list with topics in blocklist in the configuration
     *
     * @return List of filters to get blocked topics
     *
     * @throw \c ConfigurationException in case the yaml inside blocklist is bad formed
     */
    std::list<std::shared_ptr<FilterTopic>> blocklist() const;

    /**
     * @brief Return a list with topics in blocklist in the configuration
     *
     * @return List of filters to get blocked topics
     */
    std::list<ParticipantConfiguration> participants_configurations() const noexcept;


    /**
     * @brief Return a list with real topics in allowedlist in the configuration
     *
     * This method get the allowedlist in configuration and get only those topics that can actually
     * be created as Real topics.
     *
     * @todo: This method will dissapear once the dynamic module is implemented
     *
     * @return List of real topics
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is bad formed
     */
    std::set<RealTopic> real_topics() const;

protected:

    /**
     * @brief Generic method to get a list of topics from a list
     *
     * This method collects the same functionality that share \c allowlist and \c blacklist
     *
     * @param [in] list_tag: name of the tag to look the list in the configuration
     * @return List of filter topics
     */
    std::list<std::shared_ptr<FilterTopic>> generic_get_topic_list_(
            const char* list_tag) const;

    //! Yaml object with the configuration info
    const RawConfiguration raw_configuration_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_ */
