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
 * @file Track.cpp
 *
 */

#include <databroker/communication/Track.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

Track::Track(
        const RealTopic& topic,
        std::shared_ptr<IDatabrokerReader> reader,
        std::map<ParticipantId, std::shared_ptr<IDatabrokerWriter>>&& writers,
        bool enable /* = false */)
    : topic_(topic)
    , reader_(reader)
    , writers_(writers)
{
    // TODO
}

Track::~Track()
{
    // TODO
}

void Track::enable()
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

void Track::disable()
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */
