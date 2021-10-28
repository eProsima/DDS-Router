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
 * @file DatabrokerTopic.hpp
 */

#ifndef _DATABROKER_TYPES_TOPIC_DATABROKERTOPIC_HPP_
#define _DATABROKER_TYPES_TOPIC_DATABROKERTOPIC_HPP_

#include <string>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
struct DatabrokerTopic
{
    DatabrokerTopic(
            std::string topic_name,
            std::string topic_type);

    virtual ~DatabrokerTopic();

    const std::string& topic_name() const;

    const std::string& topic_type() const;

    // OPERATOR OVERLOAD
    bool operator ==(
            const DatabrokerTopic& other) const;

    bool operator <(
            const DatabrokerTopic& other) const;
protected:

    std::string topic_name_;
    std::string topic_type_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_TOPIC_DATABROKERTOPIC_HPP_ */
