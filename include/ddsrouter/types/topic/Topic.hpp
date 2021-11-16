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

#ifndef _DDSROUTER_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_
#define _DDSROUTER_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_

#include <iostream>
#include <string>

namespace eprosima {
namespace ddsrouter {

/**
 * Generic class that contains all the data required by a DDSRouter Topic
 */
struct Topic
{
    /**
     * Std constructor by topic name and topic type name
     */
    Topic(
            std::string topic_name,
            std::string topic_type) noexcept;

    //! Copy constructor
    void operator =(
            const Topic& other);

    //! Destructor
    virtual ~Topic();

    //! Topic name getter
    const std::string& topic_name() const;

    //! Topic type name getter
    const std::string& topic_type() const;

    // OPERATOR OVERLOAD
    /**
     * Equal operator
     *
     * It compares that the topic name and topic type are equal
     */
    bool operator ==(
            const Topic& other) const;

    /**
     * Minor operator
     *
     * It compares first the topic name, and if it is the same, it compares the topic type
     */
    bool operator <(
            const Topic& other) const;

protected:

    //! Topic name
    std::string topic_name_;

    //! Topic type name
    std::string topic_type_;
};

/**
 * Serialization method
 *
 * It prints the topic name and type inside "{}" and each inside "<>"
 * Example: {<TopicName>:<TopicType>}
 */
std::ostream& operator <<(
        std::ostream& os,
        const Topic& a);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_TOPIC_DDS_ROUTERTOPIC_HPP_ */
