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
 * @file Bridge.cpp
 *
 */

#include <databroker/communication/Bridge.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

Bridge::Bridge(
        const RealTopic& topic,
        std::shared_ptr<ParticipantDatabase> participant_database)
    : topic_(topic)
    , participants_(participant_database)
{
    // TODO
}

Bridge::~Bridge()
{
    // TODO
}

ReturnCode Bridge::enable()
{
    // TODO
    throw UnsupportedException("Bridge::enable not supported yet");
}

ReturnCode Bridge::disable()
{
    // TODO
    throw UnsupportedException("Bridge::disable not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */
