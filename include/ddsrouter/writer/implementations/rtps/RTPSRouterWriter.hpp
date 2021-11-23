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
 * @file RTPSRouterWriter.hpp
 */

#ifndef _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_ECHOWRITER_HPP_
#define _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_ECHOWRITER_HPP_

#include <atomic>

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/writer/implementations/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class RTPSRouterWriter : public BaseWriter
{
public:

    RTPSRouterWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant);

    virtual ~RTPSRouterWriter();

protected:

    // Specific enable/disable do not need to be implemented

    virtual ReturnCode write_(
            std::unique_ptr<DataReceived>& data) noexcept override;

    /////
    // RTPS specific methods

    fastrtps::rtps::HistoryAttributes history_attributes_() const noexcept;
    fastrtps::rtps::WriterAttributes writer_attributes_() const noexcept;
    fastrtps::TopicAttributes topic_attributes_() const noexcept;
    fastrtps::WriterQos writer_qos_() const noexcept;

    /////
    // VARIABLES

    fastrtps::rtps::RTPSWriter* rtps_writer_;
    fastrtps::rtps::WriterHistory* rtps_history_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_ECHOWRITER_HPP_ */
