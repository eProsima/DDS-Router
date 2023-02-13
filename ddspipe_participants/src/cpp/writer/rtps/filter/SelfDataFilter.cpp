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


#include <fastrtps/rtps/common/CacheChange.h>
#include <fastrtps/rtps/common/Guid.h>
#include <cpp_utils/Log.hpp>

#include <types/dds/RouterCacheChange.hpp>
#include <ddspipe_participants/writer/rtps/filter/SelfDataFilter.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

bool SelfDataFilter::is_relevant(
        const fastrtps::rtps::CacheChange_t& change,
        const fastrtps::rtps::GUID_t& reader_guid) const
{
    // It is relevant only if the reader does not belong to same participant as writer
    return change.writerGUID.guidPrefix != reader_guid.guidPrefix;
}

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
