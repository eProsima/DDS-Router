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
 * @file DDSRouterImpl.hpp
 */

#ifndef __SRC__SRC_DDSROUTERCORE_CORE_DDSROUTERIMPL_HPP_
#define __SRC__SRC_DDSROUTERCORE_CORE_DDSROUTERIMPL_HPP_

#include <atomic>
#include <map>
#include <mutex>

#include <communication/Bridge.hpp>
#include <dynamic/AllowedTopicList.hpp>
#include <dynamic/DiscoveryDatabase.hpp>
#include <library/library_dll.h>
#include <participant/IParticipant.hpp>
#include <core/ParticipantsDatabase.hpp>
#include <core/ParticipantFactory.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * TODO
 */
class DDSRouterImpl
{
public:

    /**
     * @brief Construct a new DDSRouterImpl object
     *
     * Initialize a whole DDSRouterImpl:
     * - Create its associated AllowedTopicList
     * - Create Participants and add them to \c ParticipantsDatabase
     * - Create the Bridges for RealTopics as disabled (TODO: remove when discovery is ready)
     *
     * @param [in] configuration : Configuration for the new DDS Router
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     * @throw \c InitializationException in case \c IParticipants , \c IWriters or \c IReaders creation fails.
     */
    DDSRouterImpl(
            const configuration::DDSRouterConfiguration& configuration);

    /**
     * @brief Destroy the DDSRouterImpl object
     *
     * Stop the DDSRouterImpl
     * Destroy all Bridges
     * Destroy all Participants
     */
    virtual ~DDSRouterImpl();

    // EVENTS
    /**
     * @brief Reload the allowed topic configuration
     *
     * @param [in] configuration : new configuration
     *
     * @return \c RETCODE_OK if configuration has been updated correctly
     * @return \c RETCODE_NO_DATA if new configuration has not changed
     * @return \c RETCODE_ERROR if any other error has occurred
     *
     * @throw \c ConfigurationException in case the new yaml is not well-formed
     */
    utils::ReturnCode reload_configuration(
            const configuration::DDSRouterReloadConfiguration& configuration);

    /**
     * @brief Start communication in DDS Router
     *
     * Enable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    utils::ReturnCode start() noexcept;

    /**
     * @brief Stop communication in DDS Router
     *
     * Disable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    utils::ReturnCode stop() noexcept;

protected:

    /**
     * @brief Internal Start method
     *
     * Enable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK if ok
     * @return \c RETCODE_PRECONDITION_NOT_MET if Router was not disabled
     */
    utils::ReturnCode start_() noexcept;

    /**
     * @brief Internal Stop method
     *
     * Disable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK if ok
     * @return \c RETCODE_PRECONDITION_NOT_MET if Router was not enabled
     */
    utils::ReturnCode stop_() noexcept;

    /////
    // INTERNAL INITIALIZATION METHODS

    /**
     * @brief Load allowed topics from configuration
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     */
    void init_allowed_topics_();

    /**
     * @brief  Create participants and add them to the participants database
     *
     * @throw \c ConfigurationException in case a Participant is not well configured (e.g. No kind)
     * @throw \c InitializationException in case \c IParticipants creation fails.
     */
    void init_participants_();

    /**
     * @brief  Create a disabled bridge for every real topic
     */
    void init_bridges_();

    /////
    // INTERNAL AUXILIAR METHODS

    /**
     * @brief Method called every time a new endpoint has been discovered/updated
     *
     * This method is called with the topic of a new/updated \c Endpoint discovered.
     * If the DDSRouterImpl is enabled, the new Bridge is created and enabled.
     *
     * @note This is the only method that adds topics to \c current_topics_
     *
     * @param [in] topic : topic discovered
     */
    void discovered_topic_(
            const types::RealTopic& topic) noexcept;

    /**
     * @brief Method called every time a new endpoint has been discovered/updated
     *
     * This method calls \c discovered_topic_ with the topic of \c endpoint as parameter.
     *
     * @param [in] endpoint : endpoint discovered
     */
    void discovered_endpoint_(
            const types::Endpoint& endpoint) noexcept;

    /**
     * @brief Create a new \c Bridge object
     *
     * It is created enabled if the DDSRouterImpl is enabled.
     *
     * @param [in] topic : new topic
     */
    void create_new_bridge(
            const types::RealTopic& topic,
            bool enabled = false) noexcept;

    /**
     * @brief Enable a specific topic
     *
     * If the topic did not exist before, the Bridge is created.
     *
     * @param [in] topic : Topic to be enabled
     */
    void activate_topic_(
            const types::RealTopic& topic) noexcept;

    /**
     * @brief Disable a specific topic.
     *
     * If the Bridge of the topic does not exist, do nothing.
     *
     * @param [in] topic : Topic to be disabled
     */
    void deactivate_topic_(
            const types::RealTopic& topic) noexcept;

    /**
     * @brief Activate all Topics that are allowed by the allowed topics list
     */
    void activate_all_topics_() noexcept;

    /**
     * @brief Disable all Bridges
     */
    void deactivate_all_topics_() noexcept;

    /////
    // DATA STORAGE

    /**
     * @brief  Common payload pool where every payload will be stored
     *
     * This payload will be shared by every endpoint.
     * Every reader will store its data in the pool, the track will pass this
     * data to the writers, that will release it after used.
     */
    std::shared_ptr<PayloadPool> payload_pool_;

    /**
     * @brief Object that stores every Participant running in the DDSRouterImpl
     */
    std::shared_ptr<ParticipantsDatabase> participants_database_;

    /**
     * @brief Common discovery database
     *
     * This object is shared by every Participant.
     * Every time an endpoint is discovered by any Participant, it should be
     * added to the database.
     */
    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    //! Map of bridges indexed by their topic
    std::map<types::RealTopic, std::unique_ptr<Bridge>> bridges_;

    /**
     * @brief List of topics discovered
     *
     * Every topic discovered would be added to this map.
     * If the value is true, it means this topic is currently activated.
     */
    std::map<types::RealTopic, bool> current_topics_;

    //! DDSRouterImpl configuration
    configuration::DDSRouterConfiguration configuration_;

    //! List of allowed and blocked topics
    AllowedTopicList allowed_topics_;

    //! Participant factory instance
    ParticipantFactory participant_factory_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DDSRouterImpl is currently communicating data or not
    std::atomic<bool> enabled_;

    //! Internal mutex for concurrent calls
    std::recursive_mutex mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC__SRC_DDSROUTERCORE_CORE_DDSROUTERIMPL_HPP_ */
