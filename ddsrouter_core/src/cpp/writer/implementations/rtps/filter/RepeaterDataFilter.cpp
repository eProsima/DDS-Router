// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RepeaterDataFilter.cpp
 */

#include <fastrtps/rtps/common/CacheChange.h>
#include <fastrtps/rtps/common/Guid.h>
#include <ddsrouter_utils/Log.hpp>

#include <types/dds/RouterCacheChange.hpp>
#include <writer/implementations/rtps/filter/RepeaterDataFilter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

bool RepeaterDataFilter::is_relevant(
        const fastrtps::rtps::CacheChange_t& change,
        const fastrtps::rtps::GUID_t& reader_guid) const
{
    if (!GuidDataFilter::is_relevant(change, reader_guid))
    {
        logDebug(
            REPEATER_DATA_FILTER,
            "Ignoring message by GuidDataFilter is_relevant result.");

        // If origin filter does not pass this change, it is discarded
        return false;
    }

    // As ChangePool is our own, and we use RouterCacheChange, we can cast this without problem
    const auto& change_ref = static_cast<const types::RouterCacheChange&>(change);

    bool is_relevant = change_ref.last_writer_guid_prefix != reader_guid.guidPrefix;

    logDebug(
        REPEATER_DATA_FILTER,
        "Evaluating whether Change with origin writer GUID prefix " << change_ref.last_writer_guid_prefix  <<
            " is relevant for reader GUID " << reader_guid << "? " << (is_relevant ? "TRUE" : "FALSE"));

    return is_relevant;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
