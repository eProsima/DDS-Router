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

#include <iostream>
#include <string>

#include <cpp_utils/Formatter.hpp>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/configuration/IConfiguration.hpp>
#include <ddspipe_core/types/topic/TopicInternalTypeDiscriminator.hpp>
#include <ddspipe_core/interface/ITopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

/**
 * Generic data struct that represents a Topic of data flow in the Router.
 */
struct Topic : public ITopic , public IConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default constructor
    DDSPIPE_CORE_DllAPI Topic() = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    /**
     * @brief Specialize parent compare operator
     *
     * This is specialized so if the comparison object is a std Topic it is compared as it.
     */
    DDSPIPE_CORE_DllAPI virtual bool operator==(const ITopic& other) const noexcept override;

    DDSPIPE_CORE_DllAPI virtual bool operator<(const ITopic& other) const noexcept override;

    /////////////////////////
    // METHODS
    /////////////////////////

    //! ITopic name
    DDSPIPE_CORE_DllAPI virtual std::string topic_name() const noexcept override;

    DDSPIPE_CORE_DllAPI virtual TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept override;

    DDSPIPE_CORE_DllAPI virtual bool is_valid(
        utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // METHODS TO OVERRIDE
    /////////////////////////

    //! ITopic unique name in processs
    DDSPIPE_CORE_DllAPI virtual std::string topic_unique_name() const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    /**
     * @brief Topic name
     *
     * @note it is called with m_ because the name \c topic_name was already in used by parent.
     */
    std::string m_topic_name{};

    /**
     * This refers to an internal used identifier that declares which kind of data type is going to be
     * transmitted in this topic inside the core.
     *
     * @note it is called with m_ because the name \c topic_name was already in used by parent.
     */
    TopicInternalTypeDiscriminator m_internal_type_discriminator{INTERNAL_TOPIC_TYPE_NONE};
};

/**
 * Serialization method for \c Topic object.
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Topic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
