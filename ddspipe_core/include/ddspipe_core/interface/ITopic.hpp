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
#include <ddspipe_core/types/topic/TopicInternalTypeDiscriminator.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * Generic data struct that represents an ITopic of data flow in the Router.
 */
DDSPIPE_CORE_DllAPI class ITopic
{
public:

    /**
     * @brief Virtual dtor to allow inheritance.
     */
    virtual ~ITopic() = default;

    virtual bool operator < (
            const ITopic& other) const noexcept = 0;

    virtual bool operator == (
            const ITopic& other) const noexcept = 0;

    //! ITopic name
    virtual std::string topic_name() const noexcept = 0;

    //! ITopic unique name in processs
    virtual std::string topic_unique_name() const noexcept = 0;

    /**
     * This refers to an internal used identifier that declares which kind of data type is going to be
     * transmitted in this Itopic inside the core.
     */
    virtual types::TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept = 0;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
