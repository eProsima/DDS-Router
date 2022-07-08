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
#include <#include <ddsrouter_core/types/dds/RouterCacheChange.hpp>
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
    const fastrtps::rtps::GUID_t& reader_guid) const
{

    const auto& change_ref = static_cast<const types::RouterCacheChange_t&>(change);

    bool is_relevant = change_ref.originalWriterGuidPrefix != reader_guid.guidPrefix;
    
    logDebug(REPEATER_DATA_FILTER, "Evaluating whether Change with origin writer GUID prefix " << change_ref.originalWriterGuidPrefix  << " is relevant for reader GUID " << reader_guid << "? " << (is_relevant ? "TRUE" : "FALSE"));

    return is_relevant;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
