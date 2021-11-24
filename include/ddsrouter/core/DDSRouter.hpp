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
 * @file DDSRouter.hpp
 */

#ifndef _DDSROUTER_CORE_DDSROUTER_HPP_
#define _DDSROUTER_CORE_DDSROUTER_HPP_

#include <atomic>
#include <map>
#include <mutex>

#include <ddsrouter/communication/Bridge.hpp>
#include <ddsrouter/configuration/Configuration.hpp>
#include <ddsrouter/dynamic/AllowedTopicList.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/ParticipantsDatabase.hpp>
#include <ddsrouter/participant/ParticipantFactory.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class DDSRouter
{
public:

    /**
     * @brief Construct a new DDSRouter object
     *
     * Initialize a whole DDSRouter:
     * - Create its associated AllowedTopicList
     * - Create Participants and add them to \c ParticipantsDatabase
     * - Create the Bridges for RealTopics as disabled (TODO: remove when discovery is ready)
     *
     * @param [in] configuration : Configuration for the new DDS Router
     *
     * @throw \c ConfigurationException in case the yaml inside allowedlist is not well-formed
     * @throw \c InitializationException in case \c IParticipants , \c IWriters or \c IReaders creation fails.
     */
    DDSRouter(
            const Configuration& configuration);

    /**
     * @brief Destroy the DDSRouter object
     *
     * Stop the DDSRouter
     * Destroy all Bridges
     * Destroy all Participants
     */
    virtual ~DDSRouter();

    // EVENTS
    /**
     * @brief Reload the allowed topic configuration
     *
     * @warning Unsupported
     * @todo Add noexcept when method ready
     *
     * @param [in] configuration : new configuration
     *
     * @return \c RETCODE_OK if configuration has been updated correctly
     * @return \c RETCODE_NO_DATA if new configuration has not changed
     * @return \c RETCODE_BAD_PARAMETER if configuration is not well-formed
     * @return \c RETCODE_ERROR if any other error has occurred
     */
    ReturnCode reload_configuration(
            const Configuration& configuration);

    /**
     * @brief Start communication in DDS Router
     *
     * Enable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    ReturnCode start() noexcept;

    /**
     * @brief Stop communication in DDS Router
     *
     * Disable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    ReturnCode stop() noexcept;

protected:

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
     * @throw \c InitializationException in case \c IParticipants creation fails.
     */
    void init_participants_();

    /**
     * @brief  Create a disabled bridge for every real topic
     *
     * @throw \c InitializationException in case \c IWriters or \c IReaders creation fails.
     */
    void init_bridges_();

    /////
    // INTERNAL AUXILIAR METHODS

    /**
     * @brief Method called every time a new discovery endpoint has been discovered/updated
     *
     * This method is called with the topic of a new/updated \c Endpoint discovered.
     * If the DDSRouter is enabled, the new Bridge is created and enabled.
     *
     * @note This is the only method that adds topics to \c current_topics_
     *
     * @param [in] topic : topic discovered
     */
    void discovered_topic_(
            const RealTopic& topic) noexcept;

    /**
     * @brief Create a new \c Bridge object
     *
     * It is created enabled if the DDSRouter is enabled.
     *
     * @param [in] topic : new topic
     */
    void create_new_bridge(
            const RealTopic& topic,
            bool enabled = false) noexcept;

    /**
     * @brief Enable a specific topic
     *
     * If the topic did not exist before, the Bridge is created.
     *
     * @param [in] topic : Topic to be enabled
     */
    void active_topic_(
            const RealTopic& topic) noexcept;

    /**
     * @brief Disable a specific topic.
     *
     * If the Bridge of the topic does not exist, do nothing.
     *
     * @param [in] topic : Topic to be disabled
     */
    void deactive_topic_(
            const RealTopic& topic) noexcept;

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
     * @brief Object that stores every Participant running in the DDSRouter
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
    std::map<RealTopic, std::unique_ptr<Bridge>> bridges_;

    /**
     * @brief List of topics discovered
     *
     * Every topic discovered would be added to this map.
     * If the value is true, it means this topic is currently activated.
     */
    std::map<RealTopic, bool> current_topics_;

    //! DDSRouter configuration
    Configuration configuration_;

    //! List of allowed and blocked topics
    AllowedTopicList allowed_topics_;

    //! Participant factory instance
    ParticipantFactory participant_factory_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DDSRouter is currently communicating data or not
    std::atomic<bool> enabled_;

    //! Internal mutex for concurrent calls
    std::recursive_mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CORE_DDSROUTER_HPP_ */
