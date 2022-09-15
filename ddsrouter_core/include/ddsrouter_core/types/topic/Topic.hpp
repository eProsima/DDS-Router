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

#ifndef _DDSROUTERCORE_TYPES_TOPIC_TOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_TOPIC_HPP_

#include <iostream>
#include <string>

#include <ddsrouter_utils/Formatter.hpp>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Generic Data Struct that represent a Topic of data flow in the Router
 */
struct Topic
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI Topic() = default;

    //! Construct a Topic with name
    DDSROUTER_CORE_DllAPI Topic(
            const std::string& topic_name) noexcept;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI virtual bool is_valid(utils::Formatter& error_msg) const noexcept;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI bool operator< (const Topic& other) const noexcept;

    DDSROUTER_CORE_DllAPI bool operator== (const Topic& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Topic name
    std::string topic_name;
};

/**
 * Serialization method for \c Topic object.
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Topic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_TOPIC_HPP_ */
