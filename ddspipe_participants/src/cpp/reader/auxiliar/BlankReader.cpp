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

#include <cpp_utils/exception/UnsupportedException.hpp>

#include <ddspipe_participants/reader/auxiliar/BlankReader.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

void BlankReader::enable() noexcept
{
}

void BlankReader::disable() noexcept
{
}

void BlankReader::set_on_data_available_callback(
        std::function<void()>) noexcept
{
}

void BlankReader::unset_on_data_available_callback() noexcept
{
}

utils::ReturnCode BlankReader::take(
        std::unique_ptr<core::IRoutingData>& /* data */) noexcept
{
    return utils::ReturnCode::RETCODE_NO_DATA;
}

core::types::Guid BlankReader::guid() const
{
    throw utils::UnsupportedException("guid method not allowed for non RTPS readers.");
}

fastrtps::RecursiveTimedMutex& BlankReader::get_rtps_mutex() const
{
    throw utils::UnsupportedException("get_rtps_mutex method not allowed for non RTPS readers.");
}

uint64_t BlankReader::get_unread_count() const
{
    throw utils::UnsupportedException("get_unread_count method not allowed for non RTPS readers.");
}

core::types::DdsTopic BlankReader::topic() const
{
    throw utils::UnsupportedException("topic method not allowed for non RTPS readers.");
}

core::types::ParticipantId BlankReader::participant_id() const
{
    throw utils::UnsupportedException("participant_id method not allowed for non RTPS readers.");
}

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
