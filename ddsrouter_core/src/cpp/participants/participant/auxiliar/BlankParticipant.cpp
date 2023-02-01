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
 * @file BlankParticipant.cpp
 */

#include <ddsrouter_core/participants/participant/auxiliar/BlankParticipant.hpp>
#include <ddsrouter_core/participants/reader/auxiliar/BlankReader.hpp>
#include <ddsrouter_core/participants/writer/auxiliar/BlankWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

using namespace eprosima::ddsrouter::core::types;

BlankParticipant::BlankParticipant(
        const ParticipantId& id)
    : id_(id)
{
}

ParticipantId BlankParticipant::id() const noexcept
{
    return id_;
}

bool BlankParticipant::is_repeater() const noexcept
{
    return false;
}

bool BlankParticipant::is_rtps_kind() const noexcept
{
    return false;
}

std::shared_ptr<core::IWriter> BlankParticipant::create_writer(
        DdsTopic topic)
{
    return std::make_shared<BlankWriter>();
}

std::shared_ptr<core::IReader> BlankParticipant::create_reader(
        DdsTopic topic)
{
    return std::make_shared<BlankReader>();
}

void BlankParticipant::delete_writer(
        std::shared_ptr<core::IWriter> writer) noexcept
{
}

void BlankParticipant::delete_reader(
        std::shared_ptr<core::IReader> reader) noexcept
{
}

void BlankParticipant::start()
{
    // Do nothing
}

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */
