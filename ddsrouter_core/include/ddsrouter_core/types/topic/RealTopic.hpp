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
 * @file RealTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_REALTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_REALTOPIC_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Class that represents a real topic working in a DDS network
 */
struct RealTopic : public Topic
{
    //! Constructor by topic name, topic type name, and (optionally) topic with key and topic kind
    DDSROUTER_CORE_DllAPI RealTopic(
            const std::string& topic_name,
            const std::string& topic_type,
            bool topic_with_key = false,
            bool topic_reliable = false) noexcept;

    //! Constructor by topic kind, topic name and topic type name
    DDSROUTER_CORE_DllAPI RealTopic(
            bool topic_reliable,
            const std::string& topic_name,
            const std::string& topic_type) noexcept;

    //! Default non valid topic
    DDSROUTER_CORE_DllAPI RealTopic();

    /**
     * Whether a topic name and topic type name could be used as a valid real DDS topic
     *
     * @param topic_name: topic name
     * @param type_name: topic type name
     *
     * @return: true if a topic can be formed by these values
     */
    DDSROUTER_CORE_DllAPI static bool is_real_topic(
            const std::string& topic_name,
            const std::string& type_name) noexcept;

    DDSROUTER_CORE_DllAPI bool is_valid() const noexcept override;

    //! Topic reliability getter
    DDSROUTER_CORE_DllAPI bool topic_reliable() const;

    //! Topic reliability setter
    DDSROUTER_CORE_DllAPI void topic_reliable(
            bool new_val);

protected:

    static const char* INVALID_TOPIC_NAME;  // __invalid_topic_name__
    static const char* INVALID_TOPIC_TYPE;  // __invalid_topic_type_name__

    //! The DataReader subscribed to this topic is configured as RELIABLE
    bool topic_reliable_;
};

/**
 * Serialization method
 *
 * It prints the topic name, type, kind and reliability inside "{}" and separated by ";"
 * Example: {TopicName;TopicType;no_key;reliable}
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const RealTopic& a);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_REALTOPIC_HPP_ */
