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
 * @file DatabrokerParticipantFactory.cpp
 *
 */

#include <databroker/participant/DatabrokerParticipantFactory.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

DatabrokerParticipantFactory::~DatabrokerParticipantFactory()
{
}

std::shared_ptr<IDatabrokerParticipant> DatabrokerParticipantFactory::create_participant(
        ParticipantId,
        RawConfiguration,
        std::shared_ptr<PayloadPool>,
        std::shared_ptr<DiscoveryDatabase>)
{
    // TODO
    throw UnsupportedException("DatabrokerParticipantFactory::create_participant not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */
