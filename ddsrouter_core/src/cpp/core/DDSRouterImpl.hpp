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

#include <library/library_dll.h>
#include <dynamic/DiscoveryDatabase.hpp>
#include <participant/IParticipant.hpp>
#include <participant/ParticipantsRegistry.hpp>
#include <communication/payload_pool/TopicPayloadPoolRegistry.hpp>
#include <communication/ThreadPool.hpp>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/topic/TopicKeyMap.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Implementation class of DDSRouter
 */
class DDSRouterImpl
{
public:

    /**
     * @brief Construct a new DDSRouterImpl object
     *
     * Initialize a whole DDSRouterImpl:
     * - Create its associated AllowedTopicList
     * - Create Participants and add them to \c ParticipantsRegistry
     * - Bind Participants Writers and Readers from givem Topics
     *
     * @param [in] configuration : Configuration for the new DDS Router
     *
     * @throw \c ConfigurationException in case the yaml inside allowlist is not well-formed
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
     * Enable every topic.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    utils::ReturnCode start() noexcept;

    /**
     * @brief Stop communication in DDS Router
     *
     * Disable every topic.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    utils::ReturnCode stop() noexcept;

protected:

    /////
    // INTERNAL INITIALIZATION METHODS

    /**
     * @brief  Create new participants in the participants registry
     *
     * @throw \c ConfigurationException in case a Participant is not well configured (e.g. No kind)
     * @throw \c InitializationException in case \c IParticipants creation fails.
     */
    void init_participants_();

    /////
    // INTERNAL AUXILIAR METHODS

    /**
     * @brief  Register a topic existing in the router_configuration_
     *
     * @param [in] topic : Topic to register
     *
     * Register a topic if not previously registered, possibly creating a IPayloadPool
     * If the DDSRouterImpl is enabled and the topic is allowed, then the topic is enabled.
     *
     * @throw InconsistencyException if topic has not been inserted in router_configuration_
     * @throw InconsistencyException if there is a failure getting the payload pool
     * @throw InconsistencyException if called more than once for the same topic
     */
    void register_topic_(
            const types::RealTopic& topic);

    /**
     * @brief Method called every time a new endpoint has been discovered/updated
     *
     * This method calls \c register_topic_ with the topic of \c endpoint as parameter if this topic was not registered before.
     *
     * @param [in] endpoint : endpoint discovered
     */
    void discovered_endpoint_(
            const types::Endpoint& endpoint) noexcept;

    /**
     * @brief Enable a specific topic
     *
     * If the topic did not exist before, the Bridge is created.
     *
     * @param [in] topic : Topic to be enabled
     *
     * @return \c RETCODE_OK if topic was enabled after being disabled
     * @return \c RETCODE_NOT_ENABLED if topic was already enabled
     */
    utils::ReturnCode enable_topic_(
            const types::RealTopic& topic) noexcept;

    /**
     * @brief Disable a specific topic.
     *
     * @param [in] topic : Topic to be disabled
     *
     * @return \c RETCODE_OK if topic was disabled after being enabled
     * @return \c RETCODE_NOT_ENABLED if topic was already disabled
     */
    utils::ReturnCode disable_topic_(
            const types::RealTopic& topic) noexcept;

    /**
     * @brief Activate all Topics that are allowed by the allowed topics list
     */
    void enable_all_topics_() noexcept;

    /**
     * @brief Disable all Topics regardless allow or block lists
     */
    void disable_all_topics_() noexcept;

    /////
    // DATA MEMBERS
    // NOTE: Beware of the order of the following members, as destruction order must be correct

    //! DDSRouterImpl configuration
    configuration::DDSRouterConfiguration router_configuration_;

    //! Registry storing all payload pools, from which payload pools are constructed
    fastrtps::rtps::recycle::TopicPayloadPoolRegistry payload_pool_registry_;

    //! Thread-safe registry for storing owned Participants running in the DDSRouterImpl
    ParticipantsRegistry participants_registry_;

    //! Iterable container having non-owned participants, just a proxy of participants_registry_ to iterate on participants when enabling/disabling topics.
    types::ParticipantKeySet<IParticipant*> participants_iterable_;

    /**
     * @brief Common discovery database
     *
     * This object is referenced by every Participant.
     * Every time an endpoint is discovered by any Participant, it should be
     * added to the database.
     */
    DiscoveryDatabase discovery_database_;

    /**
     * @brief Object that stores the data forward tasks
     */
    DataForwardQueue data_forward_queue_;

    //! Thread pool of workers running DataForwardTask instances
    ThreadPool workers_thread_pool_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DDSRouterImpl is currently communicating data or not
    std::atomic<bool> router_enabled_;

    //! Internal mutex for concurrent calls
    std::recursive_mutex mutex_;

    std::shared_ptr<utils::SlotThreadPool> thread_pool_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC__SRC_DDSROUTERCORE_CORE_DDSROUTERIMPL_HPP_ */
