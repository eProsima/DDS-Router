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
 * @file GuidDataFilter.cpp
 */

#include <fastrtps/rtps/common/CacheChange.h>
#include <fastrtps/rtps/common/Guid.h>
#include <ddsrouter_utils/Log.hpp>

#include <types/dds/RouterCacheChange.hpp>
#include <writer/implementations/rtps/filter/GuidDataFilter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

GuidDataFilter::GuidDataFilter(std::shared_ptr<types::GuidPrefixDataFilterType> target_guids_filter)
    : target_guids_filter(target_guids_filter)
{
    // Do nothing
}


bool GuidDataFilter::is_relevant(
        const fastrtps::rtps::CacheChange_t& change,
        const fastrtps::rtps::GUID_t& reader_guid) const
{
    std::shared_lock<types::GuidPrefixDataFilterType> lock(*target_guids_filter);
    return target_guids_filter->find(reader_guid.guidPrefix) == target_guids_filter->end();
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
