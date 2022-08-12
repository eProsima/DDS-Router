// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RPCBridge.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_RPC_RPCBRIDGE_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_RPC_RPCBRIDGE_HPP_

#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <shared_mutex>

#include <communication/Bridge.hpp>

#include <communication/rpc/ServiceRegistry.hpp>
#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/topic/RPCTopic.hpp>
#include <reader/implementations/rtps/Reader.hpp>
#include <writer/implementations/rtps/Writer.hpp>


namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Bridge object manages the communication of a DDS Topic (or \c RealTopic ).
 * It could be seen as a channel of communication as a DDS Topic, whit several Participants that
 * could publish or subscribe in this specific Topic.
 *
 * It contains N \c Tracks that will manage each direction of the communication,
 * being N the number of Participants of this communication channel.
 */
class RPCBridge : public Bridge
{
public:

    /**
     * Bridge constructor by required values
     *
     * In Bridge construction, the inside \c Tracks are created.
     * In Bridge construction, a Writer and a Reader are created for each Participant.
     *
     * @param topic: Topic of which this Bridge manages communication
     * @param participant_database: Collection of Participants to manage communication
     * Always create disabled, manual enable required. First enable creates all endpoints
     *
     * @throw InitializationException in case \c IWriters or \c IReaders creation fails.
     */
    RPCBridge(
            const types::RPCTopic& topic,
            std::shared_ptr<ParticipantsDatabase> participants_database,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<utils::SlotThreadPool> thread_pool);

    /**
     * @brief Destructor
     *
     * Before deleting, it calls \c disable.
     * It deletes all the tracks created and all Writers and Readers.
     */
    virtual ~RPCBridge();

    /**
     * Enable bridge in case it is not enabled
     * Does nothing if it is already enabled
     *
     * Thread safe
     */
    void enable() noexcept override;

    /**
     * Disable bridge in case it is not enabled
     * Does nothing if it is disabled
     *
     * Thread safe
     */
    void disable() noexcept override;

    void discovered_service(
            const types::ParticipantId& server_participant_id,
            const types::GuidPrefix& server_guid_prefix) noexcept;

    void removed_service(
            const types::ParticipantId& server_participant_id,
            const types::GuidPrefix& server_guid_prefix) noexcept;

protected:

    void init_nts_(); // throws exception, caught in enable

    void create_proxy_server_(types::ParticipantId participant_id); // throws exception

    void create_proxy_client_(types::ParticipantId participant_id); // throws exception

    void create_slot_(std::shared_ptr<rtps::Reader> reader) noexcept;

    void data_available_(const types::Guid& reader_guid) noexcept;

    void transmit_(std::shared_ptr<rtps::Reader> reader) noexcept;

    bool servers_available_() const noexcept;

    const types::RPCTopic topic_;

    bool init_;

    // Proxy servers endpoints
    std::map<types::ParticipantId, std::shared_ptr<rtps::Reader>> request_readers_;
    std::map<types::ParticipantId, std::shared_ptr<rtps::Writer>> reply_writers_;

    // Proxy clients endpoints
    std::map<types::ParticipantId, std::shared_ptr<rtps::Reader>> reply_readers_;
    std::map<types::ParticipantId, std::shared_ptr<rtps::Writer>> request_writers_;

    // Map readers' GUIDs to their associated thread pool tasks, and also keep a task emission flag.
    std::map<types::Guid, std::pair<bool, utils::TaskId>> tasks_map_;

    // The id corresponds to that of the RTPS participant which discovered the server
    std::map<types::ParticipantId, std::shared_ptr<ServiceRegistry>> service_registries_;

    std::map<types::ParticipantId, std::set<types::GuidPrefix>> current_servers_;

    //! Mutex to prevent simultaneous calls to enable and/or disable
    std::mutex mutex_;

    std::shared_timed_mutex on_transmission_mutex_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const RPCBridge&);
};

/**
 * @brief \c Bridge to stream serialization
 *
 * This method is merely a to_string of a Bridge definition.
 * It serialize the topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const RPCBridge& bridge);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_RPC_RPCBRIDGE_HPP_ */
