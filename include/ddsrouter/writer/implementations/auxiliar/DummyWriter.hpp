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
 * @file DummyWriter.hpp
 */

#ifndef _DATABROKER_WRITER_IMPLEMENTATIONS_AUX_DUMMYWRITER_HPP_
#define _DATABROKER_WRITER_IMPLEMENTATIONS_AUX_DUMMYWRITER_HPP_

#include <mutex>

#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/types/Timestamp.hpp>
#include <ddsrouter/writer/implementations/auxiliar/EchoWriter.hpp>
#include <ddsrouter/writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {

struct DataStored
{
    Timestamp timestamp;
    Guid guid_src;
    Payload payload;
};

/**
 * TODO
 */
class DummyWriter : public EchoWriter
{
public:

    DummyWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic);

    ReturnCode write(
            std::unique_ptr<DataReceived>& data) override;

    std::vector<DataStored> data_received_ref();

protected:

    std::vector<DataStored> data_stored;

    std::mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_WRITER_IMPLEMENTATIONS_AUX_DUMMYWRITER_HPP_ */
