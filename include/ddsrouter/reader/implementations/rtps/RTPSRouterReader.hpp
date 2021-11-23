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
 * @file RTPSRouterReader.hpp
 */

#ifndef _DATABROKER_READER_IMPLEMENTATIONS_AUX_RTPSREADER_HPP_
#define _DATABROKER_READER_IMPLEMENTATIONS_AUX_RTPSREADER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/rtps/attributes/ReaderAttributes.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>

#include <ddsrouter/reader/implementations/auxiliar/BaseReader.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class RTPSRouterReader : public BaseReader
{
public:

    RTPSRouterReader(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant);

    virtual ~RTPSRouterReader();

protected:

    // Specific enable/disable do not need to be implemented

    ReturnCode take_(
            std::unique_ptr<DataReceived>& data) noexcept override;

    /////
    // RTPS specific methods

    fastrtps::rtps::HistoryAttributes history_attributes_() const noexcept;
    fastrtps::rtps::ReaderAttributes reader_attributes_() const noexcept;
    fastrtps::TopicAttributes topic_attributes_() const noexcept;
    fastrtps::ReaderQos reader_qos_() const noexcept;

    /////
    // VARIABLES

    fastrtps::rtps::RTPSReader* rtps_reader_;
    fastrtps::rtps::ReaderHistory* rtps_history_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_READER_IMPLEMENTATIONS_AUX_RTPSREADER_HPP_ */
