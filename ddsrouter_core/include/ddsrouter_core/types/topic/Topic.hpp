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
 * @file Topic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_

#include <iostream>
#include <string>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Generic class that contains all the data required by a DDSRouter Topic
 */
struct Topic
{
    /**
     * Std constructor by topic name, topic type name and (optionally) topic kind
     */
    DDSROUTER_CORE_DllAPI Topic(
            std::string topic_name,
            std::string topic_type,
            bool topic_with_key = false) noexcept;

    //! Copy constructor
    DDSROUTER_CORE_DllAPI Topic& operator =(
            const Topic& other);

    //! Topic name getter
    DDSROUTER_CORE_DllAPI const std::string& topic_name() const;

    //! Topic name setter
    DDSROUTER_CORE_DllAPI void topic_name(const std::string& topic_name);

    //! Topic type name getter
    DDSROUTER_CORE_DllAPI const std::string& topic_type() const;

    //! Topic type setter
    DDSROUTER_CORE_DllAPI void topic_type(const std::string& topic_type);

    //! Topic kind getter
    DDSROUTER_CORE_DllAPI bool topic_with_key() const;

    // OPERATOR OVERLOAD
    /**
     * Equal operator
     *
     * It compares that the topic name and topic type are equal
     */
    DDSROUTER_CORE_DllAPI bool operator ==(
            const Topic& other) const;

    /**
     * Minor operator
     *
     * It compares first the topic name, and if it is the same, it compares the topic type, and if it is the same,
     * it compares keyed attribute
     */
    DDSROUTER_CORE_DllAPI bool operator <(
            const Topic& other) const;

    DDSROUTER_CORE_DllAPI virtual bool is_valid() const noexcept;

protected:

    //! Topic name
    std::string topic_name_;

    //! Topic type name
    std::string topic_type_;

    //! Topic kind; WITH_KEY(true), NO_KEY(false)
    bool topic_with_key_;
};

/**
 * Serialization method
 *
 * It prints the topic name, type and kind inside "{}" and separated by ";"
 * Example: {TopicName;TopicType;no_key}
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Topic& a);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_ */
