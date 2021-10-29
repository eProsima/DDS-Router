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
 * @file DatabrokerParticipantFactory.hpp
 */

#ifndef _DATABROKER_PARTICIPANT_DATABROKERPARTICIPANTFACTORY_HPP_
#define _DATABROKER_PARTICIPANT_DATABROKERPARTICIPANTFACTORY_HPP_

#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/types/RawConfiguration.hpp>
#include <databroker/types/ReturnCode.hpp>

namespace eprosima {
namespace databroker {

class DatabrokerParticipantFactory
{
public:

    DatabrokerParticipantFactory() = default;

    virtual ~DatabrokerParticipantFactory();

    std::shared_ptr<IDatabrokerParticipant> create_participant(
            ParticipantId id,
            RawConfiguration participant_configuration,
            std::shared_ptr<PayloadPool> payload,
            std::shared_ptr<DiscoveryDatabase> discovery_database);
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_PARTICIPANT_DATABROKERPARTICIPANTFACTORY_HPP_ */
