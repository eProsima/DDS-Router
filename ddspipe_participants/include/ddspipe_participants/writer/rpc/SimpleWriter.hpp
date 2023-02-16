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

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <ddspipe_core/types/participant/ParticipantId.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/writer/rtps/CommonWriter.hpp>
#include <ddspipe_participants/efficiency/cache_change/CacheChangePool.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rpc {

/**
 * Base RTPS Writer concrete class that implements abstract CommonWriter one.
 */
class SimpleWriter : public rtps::CommonWriter
{
public:

    /**
     * @brief Construct a new SimpleWriter object
     *
     * Get the Attributes and TopicQoS and create the SimpleWriter History and the RTPS SimpleWriter.
     *
     * @note use protected constructor so this class is not called but from subclasses
     * (Basically make abstract class without a pure virtual function).
     *
     * @param participant_id    Router Id of the Participant that has created this SimpleWriter.
     * @param topic             Topic that this SimpleWriter subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    DDSPIPE_PARTICIPANTS_DllAPI SimpleWriter(
            const core::types::ParticipantId& participant_id,
            const core::types::DdsTopic& topic,
            const std::shared_ptr<core::PayloadPool>& payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            const bool repeater = false);

    //! Override Parent method to fill fields only required for RPC.
    DDSPIPE_PARTICIPANTS_DllAPI virtual utils::ReturnCode fill_to_send_data_(
            fastrtps::rtps::CacheChange_t* to_send_change_to_fill,
            eprosima::fastrtps::rtps::WriteParams& to_send_params,
            const core::types::RtpsPayloadData& data) const noexcept;

    //! Override Parent method to fill fields after message is sent only required for RPC.
    DDSPIPE_PARTICIPANTS_DllAPI virtual void fill_sent_data_(
            const eprosima::fastrtps::rtps::WriteParams& sent_params,
            core::types::RtpsPayloadData& data_to_fill) const noexcept;
};

} /* namespace rpc */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
