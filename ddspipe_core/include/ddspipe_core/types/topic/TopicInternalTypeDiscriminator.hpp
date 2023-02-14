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

#include <ddspipe_core/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

/**
 * @brief Reference the type that is used internally in the DDS Pipe to hold the data transmitted.
 *
 * @warning as this is a const char* it is strongly suggested to only use constexpr or static values.
 */
using TopicInternalTypeDiscriminator = std::string;

const TopicInternalTypeDiscriminator INTERNAL_TOPIC_TYPE_NONE = "";

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
