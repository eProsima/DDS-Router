// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <cpp_utils/types/Fuzzy.hpp>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/types/topic/filter/IFilterTopic.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

/**
 * Data struct that uses wildcards (*, ?) to filter a DDS Router.
 */
struct WildcardDdsFilterTopic : public IFilterTopic
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default ctor that does match with every topic.
    DDSPIPE_CORE_DllAPI WildcardDdsFilterTopic() = default;

    /////////////////////////
    // FILTER METHODS
    /////////////////////////

    //! Implement \c contains parent method.
    DDSPIPE_CORE_DllAPI virtual bool contains(
            const IFilterTopic& other) const override;

    //! Implement \c matches parent method.
    DDSPIPE_CORE_DllAPI virtual bool matches(
            const ITopic& topic) const override;

    /////////////////////////
    // SERIALIZATION METHODS
    /////////////////////////

    //! Override parent \c serialize method.
    DDSPIPE_CORE_DllAPI virtual std::ostream& serialize(
            std::ostream& os) const override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Topic name filter
    utils::Fuzzy<std::string> topic_name;

    //! Type name filter. If not set matches with all.
    utils::Fuzzy<std::string> type_name;

protected:

    /////////////////////////
    // INTERNAL METHODS
    /////////////////////////

    //! Specialization for type of topic. Only specialized for DdsTopic.
    DDSPIPE_CORE_DllAPI bool matches_(
            const DdsTopic& real_topic) const;
};

/**
 * Serialization method for \c WildcardDdsFilterTopic object.
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const WildcardDdsFilterTopic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
