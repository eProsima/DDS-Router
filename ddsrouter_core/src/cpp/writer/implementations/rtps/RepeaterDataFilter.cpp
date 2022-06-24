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
 * @file RepeaterParticipant.cpp
 */

#include <fastrtps/rtps/common/CacheChange.h>
#include <fastrtps/rtps/common/Guid.h>

#include <writer/implementations/rtps/RepeaterDataFilter.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

////////////////////////
//                    //
// RepeaterDataFilter //
//                    //
////////////////////////

bool RepeaterDataFilter::is_relevant(
    const fastrtps::rtps::CacheChange_t& change,
    const fastrtps::rtps::GUID_t& reader_guid
    ) const
{
    // First fastrtps::rtps::GuidPrefix_t::size bytes of inline_qos.data correspond to the GuidPrefix
    // Compare them with the Guid prefix of the reader. If they are equal, the change was originally written by a RTPSWriter paired with the RTPSReader, so change is not relevant. Otherwise, the change has been originally written by a distinct writer, so change is relevant.

    bool ret = std::memcmp(change.inline_qos.data, reader_guid.guidPrefix.value, fastrtps::rtps::GuidPrefix_t::size) != 0;

    logDebug(REPEATER_DATA_FILTER, "Evaluating whether Change with origin writer GUID is relevant for reader GUID " << reader_guid << "? " << (ret ? "TRUE" : "FALSE"));

    return ret;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
