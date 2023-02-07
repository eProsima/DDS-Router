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

/**
 * @file SharedData.hpp
 */

#pragma once

#include <cpp_utils/macros/macros.hpp>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/data/IRoutingData.hpp>
#include <ddsrouter_core/types/topic/TopicInternalTypeId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * TODO
 */
template<typename T>
struct SharedData : public IRoutingData
{
    std::shared_ptr<T> ref;

    static TopicInternalTypeId get_topic_internal_type_id();

    static bool is_shared_internal_type(TopicInternalTypeId internal_topic_type_id);
};

/**
 * TODO
 */
constexpr const TopicInternalTypeId INTERNAL_TOPIC_TYPE_SHARED_GENERIC_PREFIX = "shared::v0::";

template<typename T>
constexpr const TopicInternalTypeId INTERNAL_TOPIC_TYPE_SHARED_GENERIC =
    INTERNAL_TOPIC_TYPE_SHARED_GENERIC_PREFIX + TYPE_NAME(T);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
