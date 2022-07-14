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
 * @file RouterCacheChange.hpp
 */

#ifndef __SRC_DDSROUTERCORE_TYPES_DDS_ROUTERCACHECHANGE_HPP_
#define __SRC_DDSROUTERCORE_TYPES_DDS_ROUTERCACHECHANGE_HPP_

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/common/GuidPrefix_t.hpp>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Unique Id of every Endpoint
struct RouterCacheChange : public fastrtps::rtps::CacheChange_t
{
public:

    //! Using parent constructors
    using fastrtps::rtps::CacheChange_t::CacheChange_t;

    fastrtps::rtps::GuidPrefix_t last_writer_guid_prefix;
};

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_TYPES_DDS_ROUTERCACHECHANGE_HPP_ */

