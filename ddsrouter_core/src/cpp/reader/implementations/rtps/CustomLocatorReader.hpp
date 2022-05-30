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
 * @file CustomLocatorReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_CUSTOMLOCATORREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_CUSTOMLOCATORREADER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/rtps/attributes/ReaderAttributes.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>

#include <reader/implementations/rtps/Reader.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * TODO
 */
class CustomLocatorReader : public Reader
{
public:

    CustomLocatorReader(
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant);

};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_CUSTOMLOCATORREADER_HPP_ */
