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
 * @file RequestWriter.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <writer/implementations/rtps/rpc/RequestWriter.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

RequestWriter::RequestWriter(
        const ParticipantId& participant_id,
        const RPCTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        std::shared_ptr<ServiceRegistry> service_registry)
    : Writer(participant_id, topic.request_topic(), payload_pool, rtps_participant)
    , service_registry_(service_registry)
{
    logInfo(DDSROUTER_RTPS_REQUESTWRITER, "New RequestWriter created for topic " << topic_);
}

RequestWriter::~RequestWriter()
{
    logInfo(DDSROUTER_RTPS_REQUESTWRITER, "Deleting RequestWriter for topic " << topic_);
}

utils::ReturnCode RequestWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    // if (participant_id_ != service_registry_->server_participant_id())
    // {
    //     // send request only when there is a server to receive it
    //     // change return code if filtering logic is moved to Track
    //     return utils::ReturnCode::RETCODE_OK;
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
        logDevError(DDSROUTER_RTPS_REQUESTWRITER, "Error getting Payload.");
        return utils::ReturnCode::RETCODE_ERROR;
    }

    logDebug(DDSROUTER_RTPS_REQUESTWRITER,
            "Writer " << *this << " sending payload " << new_change->serializedPayload << " from " <<
            data->source_guid);

    eprosima::fastrtps::rtps::WriteParams wparams = new_change->write_params;
    SampleIdentity reply_related_sample_identity = data->write_params.sample_identity();
    reply_related_sample_identity.sequence_number(data->sequenceNumber);
    // if (reply_related_sample_identity == SampleIdentity::Unknown())
    // {
    //     logWarning();
        // return utils::ReturnCode::RETCODE_ERROR;
    // }

    // Thread safe as registry's related_sample_identity is set only in creation, when this writer is still disabled
    // Also note that concurrent \c write_ operations in the same \c RequestWriter are not possible, as \c write from parent \c BaseWriter is guarded
    wparams.related_sample_identity(service_registry_->related_sample_identity_nts());
    rtps_history_->add_change(new_change, wparams);

    service_registry_->add(new_change->sequenceNumber, {data->receiver_participant_id, reply_related_sample_identity});

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
