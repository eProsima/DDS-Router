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
 * @file Bridge.hpp
 */

#ifndef _DATABROKER_COMMUNICATION_BRIDGE_HPP_
#define _DATABROKER_COMMUNICATION_BRIDGE_HPP_

#include <databroker/communication/Track.hpp>
#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/participant/ParticipantDatabase.hpp>
#include <databroker/types/ParticipantId.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class Bridge
{
public:

    Bridge(
            RealTopic,
            std::shared_ptr<ParticipantDatabase>);

    ReturnCode enable();

    ReturnCode disable();

protected:

    const RealTopic topic_;

    const std::shared_ptr<ParticipantDatabase> participants_;

    std::map<ParticipantId, Track> tracks;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_COMMUNICATION_BRIDGE_HPP_ */
