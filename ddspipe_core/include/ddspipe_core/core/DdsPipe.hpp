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
// limitations under the License\.

#pragma once

#include <memory>

#include <cpp_utils/ReturnCode.hpp>
#include <cpp_utils/thread_pool/pool/SlotThreadPool.hpp>

#include <ddspipe_core/communication/dds/DdsBridge.hpp>
#include <ddspipe_core/communication/rpc/RPCBridge.hpp>
#include <ddspipe_core/dynamic/DiscoveryDatabase.hpp>
#include <ddspipe_core/dynamic/AllowedTopicList.hpp>
#include <ddspipe_core/dynamic/ParticipantsDatabase.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <ddspipe_core/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * TODO
 */
class DdsPipe
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    /**
     * @brief Construct a new DdsPipe object
     *
     * Initialize a whole DdsPipe:
     * - Create its associated AllowedTopicList
     * - Create Participants and add them to \c ParticipantsDatabase
     * - Create the Bridges for (allowed) builtin topics
     *
     * @param [in] configuration : Configuration for the new DDS Router
     *
     * @throw \c ConfigurationException in case the yaml inside allowlist is not well-formed
     * @throw \c InitializationException in case \c IParticipants , \c IWriters or \c IReaders creation fails.
     *
     * @todo change SlotThreadPool for a IThreadPool when exist.
     */
    DDSPIPE_CORE_DllAPI DdsPipe(
            const std::shared_ptr<AllowedTopicList>& allowed_topics,
            const std::shared_ptr<DiscoveryDatabase>& discovery_database,
            const std::shared_ptr<PayloadPool>& payload_pool,
            const std::shared_ptr<ParticipantsDatabase>& participants_database,
            const std::shared_ptr<utils::SlotThreadPool>& thread_pool,
            const std::set<utils::Heritable<types::DistributedTopic>>& builtin_topics = {});

    /**
     * @brief Destroy the DdsPipe object
     *
     * Stop the DdsPipe
     * Destroy all Bridges
     * Destroy all Participants
     */
    DDSPIPE_CORE_DllAPI ~DdsPipe();

    /////////////////////////
    // INTERACTION METHODS
    /////////////////////////

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
    DDSPIPE_CORE_DllAPI utils::ReturnCode reload_allowed_topics(
            const std::shared_ptr<AllowedTopicList>& allowed_topics);

    /////////////////////////
    // ENABLING METHODS
    /////////////////////////

    /**
     * @brief Start communication in DDS Router
     *
     * Enable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    DDSPIPE_CORE_DllAPI utils::ReturnCode start() noexcept;

    /**
     * @brief Stop communication in DDS Router
     *
     * Disable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    DDSPIPE_CORE_DllAPI utils::ReturnCode stop() noexcept;

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
     * @brief  Create a disabled bridge for every real topic
     */
    void init_bridges_(const std::set<utils::Heritable<types::DistributedTopic>>& builtin_topics);

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
            const utils::Heritable<types::DistributedTopic>& topic) noexcept;

    /**
     * @brief Method called every time a new endpoint (corresponding to a server) has been discovered/updated
     *
     * This method is called with the topic of a new/updated \c Endpoint discovered.
     * If the DDSRouterImpl is enabled and no bridge exists, the new RPCBridge is created (and enabled if allowed).
     *
     * @note This is the only method that adds topics to \c current_services_
     *
     * @param [in] topic : topic discovered
     * @param [in] server_participant_id : id of participant discovering server
     * @param [in] server_guid_prefix : GUID Prefix of discovered server
     */
    void discovered_service_(
            const types::RpcTopic& topic,
            const types::ParticipantId& server_participant_id,
            const types::GuidPrefix& server_guid_prefix) noexcept;

    /**
     * @brief Method called every time a new endpoint (corresponding to a server) has been removed/dropped
     *
     * This method is called with the topic of a removed/dropped \c Endpoint.
     *
     * @param [in] topic : topic discovered
     * @param [in] server_participant_id : id of participant discovering server
     * @param [in] server_guid_prefix : GUID Prefix of discovered server
     */
    void removed_service_(
            const types::RpcTopic& topic,
            const types::ParticipantId& server_participant_id,
            const types::GuidPrefix& server_guid_prefix) noexcept;

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
     * @brief Method called every time a new endpoint has been removed/dropped
     *
     * @param [in] endpoint : endpoint removed/dropped
     */
    void removed_endpoint_(
            const types::Endpoint& endpoint) noexcept;

    /**
     * @brief Create a new \c DdsBridge object
     *
     * It is created enabled if the DDSRouterImpl is enabled.
     *
     * @param [in] topic : new topic
     */
    void create_new_bridge_(
            const utils::Heritable<types::DistributedTopic>& topic,
            bool enabled = false) noexcept;

    /**
     * @brief Create a new \c RPCBridge object
     *
     * It is always created disabled.
     *
     * @param [in] topic : new topic
     */
    void create_new_service_(
            const types::RpcTopic& topic) noexcept;

    /**
     * @brief Enable a specific topic
     *
     * If the topic did not exist before, the Bridge is created.
     *
     * @param [in] topic : Topic to be enabled
     */
    void activate_topic_(
            const utils::Heritable<types::DistributedTopic>& topic) noexcept;

    /**
     * @brief Disable a specific topic.
     *
     * If the Bridge of the topic does not exist, do nothing.
     *
     * @param [in] topic : Topic to be disabled
     */
    void deactivate_topic_(
            const utils::Heritable<types::DistributedTopic>& topic) noexcept;

    /**
     * @brief Activate all Topics that are allowed by the allowed topics list
     */
    void activate_all_topics_() noexcept;

    /**
     * @brief Disable all Bridges
     */
    void deactivate_all_topics_() noexcept;

    /////////////////////////
    // SHARED DATA STORAGE
    /////////////////////////

    //! List of allowed and blocked topics
    std::shared_ptr<AllowedTopicList> allowed_topics_;

    /**
     * @brief Common discovery database
     *
     * This object is shared by every Participant.
     * Every time an endpoint is discovered by any Participant, it should be
     * added to the database.
     */
    std::shared_ptr<DiscoveryDatabase> discovery_database_;

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

    //! Thread Pool for tracks
    std::shared_ptr<utils::SlotThreadPool> thread_pool_;

    /////////////////////////
    // INTERNAL DATA STORAGE
    /////////////////////////

    //! Map of bridges indexed by their topic
    std::map<utils::Heritable<types::DistributedTopic>, std::unique_ptr<DdsBridge>> bridges_;

    //! Map of RPC bridges indexed by their topic
    std::map<types::RpcTopic, std::unique_ptr<RPCBridge>> rpc_bridges_;

    /**
     * @brief List of topics discovered
     *
     * Every topic discovered would be added to this map.
     * If the value is true, it means this topic is currently activated.
     */
    std::map<utils::Heritable<types::DistributedTopic>, bool> current_topics_;

    /**
     * @brief List of RPC topics discovered
     *
     * Every RPC topic discovered would is added to this map.
     * If the value is true, it means this service is allowed.
     */
    std::map<types::RpcTopic, bool> current_services_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DDSRouterImpl is currently communicating data or not
    std::atomic<bool> enabled_;

    /**
     * @brief Internal mutex for concurrent calls
     *
     * @todo this should not require to be recursive.
     */
    std::recursive_mutex mutex_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
