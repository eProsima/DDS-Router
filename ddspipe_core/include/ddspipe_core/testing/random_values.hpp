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
// limitations under the License\.

#pragma once

#include <cpp_utils/exception/InitializationException.hpp>

#include <ddspipe_core/types/dds/DomainId.hpp>
#include <ddspipe_core/types/dds/Endpoint.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace testing {

DDSPIPE_CORE_DllAPI types::Guid random_guid(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::DomainId random_domain(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::GuidPrefix random_guid_prefix(
        unsigned int seed = 0,
        bool ros = false);

DDSPIPE_CORE_DllAPI types::ParticipantId random_participant_id(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::DdsTopic random_dds_topic(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::EndpointKind random_endpoint_kind(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::Endpoint random_endpoint(
        unsigned int seed = 0);

DDSPIPE_CORE_DllAPI types::TopicQoS random_topic_qos(
        unsigned int seed = 0);

} /* namespace testing */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
