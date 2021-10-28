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
 * @file RealTopic.cpp
 *
 */

#include <databroker/types/topic/RealTopic.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace databroker {

bool RealTopic::is_real_topic(
        const std::string& topic_name,
        const std::string& type_name)
{
    // TODO
    throw UnsupportedException("RealTopic::is_real_topic not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */
