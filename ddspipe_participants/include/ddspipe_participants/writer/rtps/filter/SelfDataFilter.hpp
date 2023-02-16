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

#pragma once

#include <fastdds/rtps/interfaces/IReaderDataFilter.hpp>

#include <ddspipe_participants/library/library_dll.h>

/////
// Forward declarations
namespace eprosima {
namespace fastrtps {
namespace rtps {

struct CacheChange_t;
struct GUID_t;

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

/**
 * This filter allows to not send messages from this Writer to the Readers in the same Participant.
 */
DDSPIPE_PARTICIPANTS_DllAPI class SelfDataFilter : public fastdds::rtps::IReaderDataFilter
{
public:

    /**
     * @brief Whether incoming change is relevant for this reader.
     *
     * @return true if the reader does not belong to same Participant.
     * @return false otherwise.
     */
    bool is_relevant(
            const fastrtps::rtps::CacheChange_t& change,
            const fastrtps::rtps::GUID_t& reader_guid
            ) const override;
};

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
