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
 * @file DataForwardTask.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_DDS_DATA_HPP_
#define _DDSROUTERCORE_TYPES_DDS_DATA_HPP_

#include <fastdds/rtps/common/CacheChange.h>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/Guid.hpp>

#include <memory>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Kind of every unit that creates a Payload
using PayloadUnit = eprosima::fastrtps::rtps::octet;

//! Payload references the raw data received.
using Payload = eprosima::fastrtps::rtps::SerializedPayload_t;

//! \c octet to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const PayloadUnit& octet);

//! \c SerializedPayload_t to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Payload& payload);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_DATA_HPP_ */
