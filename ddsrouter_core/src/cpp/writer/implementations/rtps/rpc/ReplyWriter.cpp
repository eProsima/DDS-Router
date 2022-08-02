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
 * @file ReplyWriter.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <writer/implementations/rtps/rpc/ReplyWriter.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

ReplyWriter::ReplyWriter(
        const ParticipantId& participant_id,
        const RPCTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
        // std::shared_ptr<ServiceRegistry> service_registry)
    : Writer(participant_id, topic.reply_topic(), payload_pool, rtps_participant)
    // , service_registry_(service_registry)
{
    logInfo(DDSROUTER_RTPS_REPLYWRITER, "New ReplyWriter created for topic " << topic_);
}

ReplyWriter::~ReplyWriter()
{
    logInfo(DDSROUTER_RTPS_REPLYWRITER, "Deleting ReplyWriter for topic " << topic_);
}

utils::ReturnCode ReplyWriter::write(
        std::unique_ptr<DataReceived>& data,
        SampleIdentity related_sample_identity) noexcept
{
    // std::pair<ParticipantId, SampleIdentity> registry_entry = service_registry_->get(data->sequenceNumber);
    // ParticipantId request_receiver_participant_id = std::get<0>(registry_entry);
    // SampleIdentity reply_related_sample_identity = std::get<1>(registry_entry);

    // if (!request_receiver_participant_id.is_valid() || participant_id_ != request_receiver_participant_id)
    // {
    //     // if entry not found it means it has already been sent by appropiate participant
    //     // send reply only through the participant which received the request in the first place
    //     // change return code if filtering logic is moved to Track
    //     return utils::ReturnCode::RETCODE_OK;
    // }

    // if (data->write_params.sample_identity() == SampleIdentity::Unknown())
    // {
    //     logError
    //     utils::ReturnCode::RETCODE_ERROR
    // }

    // Take new Change from history
    fastrtps::rtps::CacheChange_t* new_change = rtps_writer_->new_change(eprosima::fastrtps::rtps::ChangeKind_t::ALIVE);

    // If still is not able to get a change, return an error code
    if (!new_change)
    {
        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Get the Payload (copying it)
    eprosima::fastrtps::rtps::IPayloadPool* payload_owner = payload_pool_.get();
    if (!payload_pool_->get_payload(data->payload, payload_owner, (*new_change)))
    {
        logDevError(DDSROUTER_RTPS_REPLYWRITER, "Error getting Payload.");
        return utils::ReturnCode::RETCODE_ERROR;
    }

    logDebug(DDSROUTER_RTPS_REPLYWRITER,
            "Writer " << *this << " sending payload " << new_change->serializedPayload << " from " <<
            data->source_guid);

    eprosima::fastrtps::rtps::WriteParams wparams = new_change->write_params;

    wparams.related_sample_identity(related_sample_identity);
    rtps_history_->add_change(new_change, wparams);

    /////////////////////////////////////////////////7
    // service_registry_->erase(data->sequenceNumber);
    /////////////////////////////////////////////////7

    // if (!topic_.topic_reliable() ||
    //         (topic_.qos_attached() &&
    //         topic_.qos().reliability() != fastrtps::rtps::ReliabilityKind_t::RELIABLE))
    // {
    //     // Change has been sent, remove it if best_effort (TODO: does this really work?)
    //     rtps_history_->remove_change(new_change);
    // }

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
