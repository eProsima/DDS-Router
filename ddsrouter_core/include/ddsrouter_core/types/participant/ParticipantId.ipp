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
 * @file ParticipantId.ipp
 *
 * Exposes implementations of ParticipantCompare_ templated by a HoldParticipantIdPtrT type representing a pointer to a type T exposing a non-static method T::id().
 */

#ifndef _DDSROUTERCORE_TYPES_PARTICIPANTID_IPP_
#define _DDSROUTERCORE_TYPES_PARTICIPANTID_IPP_

#include <string>
#include <array>
#include <unordered_map>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const HoldParticipantIdPtrT& ptr_a,
        const HoldParticipantIdPtrT& ptr_b) const
{
    return ptr_a->id().name() < ptr_b->id().name();
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const ParticipantId& id_a,
        const HoldParticipantIdPtrT& ptr_b) const
{
    return id_a.name() < ptr_b->id().name();
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const HoldParticipantIdPtrT& ptr_a,
        const ParticipantId& id_b) const
{
    return ptr_a->id().name() < id_b.name();
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const ParticipantName& name_a,
        const HoldParticipantIdPtrT& ptr_b) const
{
    return name_a < ptr_b->id().name();
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const HoldParticipantIdPtrT& ptr_a,
        const ParticipantName& name_b) const
{
    return ptr_a->id().name() < name_b;
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const char* name_a,
        const HoldParticipantIdPtrT& ptr_b) const
{
    return std::string(name_a) < ptr_b->id().name();
}

template <typename HoldParticipantIdPtrT>
bool ParticipantCompare_<HoldParticipantIdPtrT>::operator ()(
        const HoldParticipantIdPtrT& ptr_a,
        const char* name_b) const
{
    return ptr_a->id().name() < std::string(name_b);
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTID_IPP_ */
