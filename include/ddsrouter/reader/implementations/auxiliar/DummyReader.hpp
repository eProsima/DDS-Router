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
 * @file DummyReader.hpp
 */

#ifndef _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_
#define _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_

#include <atomic>
#include <mutex>
#include <queue>

#include <ddsrouter/reader/IReader.hpp>
#include <ddsrouter/types/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

struct DataToSend
{
    Guid guid_src;
    Payload payload;
};

/**
 * TODO
 */
class DummyReader : public IReader
{
public:

    DummyReader(
        const ParticipantId& participant_id,
        const RealTopic& topic);

    void enable() override;

    void disable() override;

    void set_on_data_available_callback(
            std::function<void()> new_callback) override;

    ReturnCode take(
            std::unique_ptr<DataReceived>& data_received) override;

    void add_message_to_send(DataToSend data);

protected:

    ParticipantId participant_id_;

    RealTopic topic_;

    std::function<void()> on_data_available_callback_;

    std::atomic<bool> enabled_;

    std::queue<DataToSend> data_to_send_;

    std::mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_ */
