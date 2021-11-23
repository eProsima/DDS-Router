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

#include <atomic>
#include <mutex>
#include <queue>

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
        std::shared_ptr<PayloadPool> payload_pool);


protected:

    // Specific disable do not need to be implemented

    void enable_() noexcept override;

    ReturnCode take_(
            std::unique_ptr<DataReceived>& data) noexcept override;

    mutable std::recursive_mutex rtps_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_READER_IMPLEMENTATIONS_AUX_RTPSREADER_HPP_ */
