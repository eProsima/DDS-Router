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
 * @file Bridge.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_BRIDGE_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_BRIDGE_HPP_

#include <core/ParticipantsDatabase.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/thread_pool/pool/SlotThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Bridge object manages the communication of a DDS Topic (or \c RPCTopic ).
 * It could be seen as a channel of communication as a DDS Topic, with several Participants that
 * could publish or subscribe in this specific Topic.
 *
 * It is implemented by \c DDSBridge and \c RPCBridge , which handle \c DdsTopic and \c RPCTopic , respectively.
 */
class Bridge
{
public:

    /**
     * Bridge constructor by required values
     *
     * @param participant_database: Collection of Participants to manage communication
     * @param payload_pool: Payload Pool that handles the reservation/release of payloads throughout the DDS Router
     * @param thread_pool: Shared pool of threads in charge of data transmission.
     *
     * @note Always created disabled. Enable in children constructors if needed.
     *
     */
    Bridge(
            std::shared_ptr<ParticipantsDatabase> participants_database,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<utils::SlotThreadPool> thread_pool);

    /**
     * Enable bridge
     */
    virtual void enable() noexcept = 0;

    /**
     * Disable bridge
     */
    virtual void disable() noexcept = 0;

protected:

    //! Collection of Participants to manage communication
    const std::shared_ptr<ParticipantsDatabase> participants_;

    //! Common shared payload pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! Common shared thread pool
    std::shared_ptr<utils::SlotThreadPool> thread_pool_;

    //! Whether the Bridge is currently enabled
    std::atomic<bool> enabled_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_BRIDGE_HPP_ */
