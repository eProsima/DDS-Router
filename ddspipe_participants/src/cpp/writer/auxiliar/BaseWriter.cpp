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

#include <cpp_utils/Log.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_participants/writer/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

BaseWriter::BaseWriter(
        const core::types::ParticipantId& participant_id,
        const std::shared_ptr<core::PayloadPool>& payload_pool)
    : participant_id_(participant_id)
    , payload_pool_(payload_pool)
    , enabled_(false)
{
    logDebug(DDSROUTER_BASEWRITER, "Creating Writer " << *this << ".");
}

void BaseWriter::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // If it is enabled, do nothing
    if (!enabled_.load())
    {
        enabled_.store(true);

        // Call specific enable
        enable_();
    }
}

void BaseWriter::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // If it is not enabled, do nothing
    if (enabled_.load())
    {
        enabled_.store(false);

        // Call specific disable
        disable_();
    }
}

utils::ReturnCode BaseWriter::write(
        core::IRoutingData& data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        return write_(data);
    }
    else
    {
        logDevError(DDSROUTER_BASEWRITER, "Attempt to write data from disabled Writer in topic in Participant " << participant_id_);
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

void BaseWriter::enable_() noexcept
{
    // It does nothing. Override this method so it has functionality.
}

void BaseWriter::disable_() noexcept
{
    // It does nothing. Override this method so it has functionality.
}

std::ostream& operator <<(
        std::ostream& os,
        const BaseWriter& writer)
{
    os << "Writer{" << writer.participant_id_ << "}";
    return os;
}

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
