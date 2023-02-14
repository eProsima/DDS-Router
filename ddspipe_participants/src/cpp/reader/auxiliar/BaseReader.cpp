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
#include <cpp_utils/Log.hpp>

#include <ddspipe_participants/reader/auxiliar/BaseReader.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

const std::function<void()> BaseReader::DEFAULT_ON_DATA_AVAILABLE_CALLBACK =
        []()
        {
            logDevError(DDSROUTER_READER, "Calling unset callback");
        };

BaseReader::BaseReader(
        const core::types::ParticipantId& participant_id,
        const std::shared_ptr<core::PayloadPool>& payload_pool)
    : participant_id_(participant_id)
    , payload_pool_(payload_pool)
    , on_data_available_lambda_(DEFAULT_ON_DATA_AVAILABLE_CALLBACK)
    , on_data_available_lambda_set_(false)
    , enabled_(false)
{
    logDebug(DDSROUTER_BASEREADER, "Creating Reader " << *this << ".");
}

void BaseReader::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // If it is enabled, do nothing
    if (!enabled_.load())
    {
        enabled_.store(true);

        // Call specific enable
        enable_nts_();
    }
}

void BaseReader::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // If it is not enabled, do nothing
    if (enabled_.load())
    {
        enabled_.store(false);

        // Call specific disable
        disable_nts_();
    }
}

void BaseReader::set_on_data_available_callback(
        std::function<void()> on_data_available_lambda) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (on_data_available_lambda_set_)
    {
        logDevError(DDSROUTER_BASEREADER, "Changing on_data_available callback for Reader in Participant " << participant_id_);
    }

    on_data_available_lambda_ = on_data_available_lambda;
    on_data_available_lambda_set_ = true;
}

void BaseReader::unset_on_data_available_callback() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!on_data_available_lambda_set_)
    {
        logDevError(DDSROUTER_BASEREADER, "Unsetting a non set on_data_available callback for Reader in Participant " << participant_id_);
    }

    on_data_available_lambda_ = DEFAULT_ON_DATA_AVAILABLE_CALLBACK;
    on_data_available_lambda_set_ = false;
}

utils::ReturnCode BaseReader::take(
        std::unique_ptr<core::IRoutingData>& data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        return take_nts_(data);
    }
    else
    {
        logDevError(DDSROUTER_BASEREADER, "Attempt to take data from disabled Reader in Participant " << participant_id_);
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

core::types::ParticipantId BaseReader::participant_id() const noexcept
{
    return participant_id_;
}

void BaseReader::on_data_available_() const noexcept
{
    if (on_data_available_lambda_set_)
    {
        on_data_available_lambda_();
    }
    else
    {
        logDevError(DDSROUTER_BASEREADER, "Calling not set on_data_available callback for Reader in Participant " << participant_id_);
    }
}

void BaseReader::enable_nts_() noexcept
{
    // It does nothing. Override this method so it has functionality.
}

void BaseReader::disable_nts_() noexcept
{
    // It does nothing. Override this method so it has functionality.
}

std::ostream& operator <<(
        std::ostream& os,
        const BaseReader& reader)
{
    os << "Reader{" << reader.participant_id_ << "}";
    return os;
}

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
