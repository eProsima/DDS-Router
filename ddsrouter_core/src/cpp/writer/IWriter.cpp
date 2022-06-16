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
 * @file IWriter.cpp
 */

#include <ddsrouter_utils/Log.hpp>
#include <writer/IWriter.hpp>
#include <reader/IReader.hpp>
#include <ddsrouter_core/types/endpoint/BaseWriterReader.ipp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

IWriter::IWriter(
        const ParticipantId& id,
        const RealTopic& topic,
        fastrtps::rtps::IPayloadPool* payload_pool)
    : BaseWriterReader<EndpointKind::writer>(id, topic)
    , payload_pool_(payload_pool)
{
    logDebug(DDSROUTER_BASEWRITER, "Creating Writer " << *this << ".");
}

IWriter::~IWriter()
{
    logDebug(DDSROUTER_BASEWRITER, "Destroying Writer " << *this << ".");
}

std::ostream& operator <<(
        std::ostream& os,
        const IWriter& writer)
{
    os << "Writer{" << writer.id().name() << ";" << writer.topic() << "}";
    return os;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
