// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/rtps/common/ChangeKind_t.hpp>
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/common/Time_t.h>
#include <fastdds/rtps/common/Time_t.h>
#include <fastdds/rtps/common/Types.h>
#include <fastdds/rtps/common/WriteParams.h>

#include <ddspipe_core/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

//! Durability kind enumeration
using PartitionQosPolicy = eprosima::fastdds::dds::PartitionQosPolicy;

//! Partition configuration
using OwnershipStrengthQosPolicy = eprosima::fastdds::dds::OwnershipStrengthQosPolicy;

//! Instance Handler type
using InstanceHandle = eprosima::fastrtps::rtps::InstanceHandle_t;

//! Instance Handler type
using ChangeKind = eprosima::fastrtps::rtps::ChangeKind_t;

//! Fast DDS Time
using DataTime = eprosima::fastrtps::rtps::Time_t;

//! Kind of every unit that creates a Payload
using PayloadUnit = eprosima::fastrtps::rtps::octet;

//! Payload references the raw data received.
using Payload = eprosima::fastrtps::rtps::SerializedPayload_t;

//! \c octet to stream serializator
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const PayloadUnit& octet);

//! \c SerializedPayload_t to stream serializator
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Payload& payload);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
