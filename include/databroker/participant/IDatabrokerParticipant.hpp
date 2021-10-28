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
 * @file IDatabrokerParticipant.hpp
 */

#ifndef _DATABROKER_PARTICIPANT_IDATABROKERPARTICIPANT_HPP_
#define _DATABROKER_PARTICIPANT_IDATABROKERPARTICIPANT_HPP_

#include <databroker/communication/PayloadPool.hpp>
#include <databroker/dynamic/DiscoveryDatabase.hpp>
#include <databroker/reader/IDatabrokerReader.hpp>
#include <databroker/types/Endpoint.hpp>
#include <databroker/types/ParticipantId.hpp>
#include <databroker/types/RawConfiguration.hpp>
#include <databroker/writer/IDatabrokerWriter.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class IDatabrokerParticipant
{
public:

    IDatabrokerParticipant(
        ParticipantId id,
        RawConfiguration,
        std::shared_ptr<PayloadPool>,
        std::shared_ptr<DiscoveryDatabase>);

    std::shared_ptr<IDatabrokerWriter> create_writer(
            RealTopic);

    std::shared_ptr<IDatabrokerReader> create_reader(
            RealTopic,
            std::function<void()> on_data_available_lambda); // lambda as listener
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_PARTICIPANT_IDATABROKERPARTICIPANT_HPP_ */
