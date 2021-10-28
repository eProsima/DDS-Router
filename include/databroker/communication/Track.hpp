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
 * @file Track.hpp
 */

#ifndef _DATABROKER_COMMUNICATION_TRACK_HPP_
#define _DATABROKER_COMMUNICATION_TRACK_HPP_

#include <databroker/communication/PayloadPool.hpp>
#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/reader/IDatabrokerReader.hpp>
#include <databroker/writer/IDatabrokerWriter.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class Track
{
public:

    Track(
            RealTopic,
            IDatabrokerParticipant source,
            std::list<std::shared_ptr<IDatabrokerParticipant>> targets);

    ReturnCode enable();

    ReturnCode disable();

protected:

    ReturnCode transmit_();

protected:

    std::shared_ptr<IDatabrokerParticipant> source_participant_;

    std::list<std::shared_ptr<IDatabrokerParticipant>> target_participants_;

    std::shared_ptr<IDatabrokerReader> reader_;

    std::map<ParticipantId, std::shared_ptr<IDatabrokerWriter>> writers_;

    std::atomic<bool> are_data_available_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_COMMUNICATION_TRACK_HPP_ */
