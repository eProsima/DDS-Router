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
 * @file IParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_

#include <ddsrouter/communication/PayloadPool.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/reader/IReader.hpp>
#include <ddsrouter/types/endpoint/Endpoint.hpp>
#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/types/ParticipantType.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class IParticipant
{
public:

    virtual ParticipantId id() const = 0;

    virtual ParticipantType type() const = 0;

    virtual std::shared_ptr<IWriter> create_writer(
            RealTopic topic) = 0;

    virtual std::shared_ptr<IReader> create_reader(
            RealTopic topic) = 0;

    virtual void delete_writer(
            std::shared_ptr<IWriter> writer) = 0;

    virtual void delete_reader(
            std::shared_ptr<IReader> reader) = 0;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_ */
