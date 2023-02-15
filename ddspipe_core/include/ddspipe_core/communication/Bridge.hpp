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

#pragma once

#include <cpp_utils/thread_pool/pool/SlotThreadPool.hpp>
#include <cpp_utils/memory/Heritable.hpp>

#include <ddspipe_core/dynamic/ParticipantsDatabase.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * Bridge object manages the communication of a DDS Topic (or \c RpcTopic ).
 * It could be seen as a channel of communication as a DDS Topic, with several Participants that
 * could publish or subscribe in this specific Topic.
 *
 * It is implemented by \c DdsBridge and \c RPCBridge , which handle \c DistributedTopic and \c RpcTopic , respectively.
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
            const std::shared_ptr<ParticipantsDatabase>& participants_database,
            const std::shared_ptr<PayloadPool>& payload_pool,
            const std::shared_ptr<utils::SlotThreadPool>& thread_pool);

    /**
     * Copy method not allowed
     *
     * Bridge creates in constructor all the inside Tracks needed, and thus it should not be copied
     */
    void operator =(
            const Bridge&) = delete;

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
    const std::shared_ptr<PayloadPool> payload_pool_;

    //! Common shared thread pool
    const std::shared_ptr<utils::SlotThreadPool> thread_pool_;

    //! Whether the Bridge is currently enabled
    std::atomic<bool> enabled_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
