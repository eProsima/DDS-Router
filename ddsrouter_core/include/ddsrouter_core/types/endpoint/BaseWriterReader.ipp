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
 * @file BaseWriterReader.ipp
 */

#ifndef __SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_IPP_
#define __SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_IPP_

#include <ddsrouter_core/types/endpoint/BaseWriterReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

template <EndpointKind EPKind>
BaseWriterReader<EPKind>::BaseWriterReader(
        const ParticipantId& id,
        const RealTopic& topic)
    : id_(id)
    , topic_(topic)
{
}

template <EndpointKind EPKind>
BaseWriterReader<EPKind>::~BaseWriterReader()
{
}

template <EndpointKind EPKind>
const ParticipantId& BaseWriterReader<EPKind>::id() const noexcept
{
    return id_;
}

template <EndpointKind EPKind>
const RealTopic& BaseWriterReader<EPKind>::topic() const noexcept
{
    return topic_;
}

} // namespace types
} // namespace core
} // namespace ddsrouter
} // namespace eprosima

#endif /*__SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_IPP_ */
