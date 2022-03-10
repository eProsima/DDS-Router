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
 * @file WildcardTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_WILDCARDTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_WILDCARDTOPIC_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Concrete class that represents an FilterTopic that uses the special char "*" as any substring
 */
class WildcardTopic : public FilterTopic
{
public:

    //! Using parent constructos
    using FilterTopic::FilterTopic;

    //! Constructor that allows any type
    DDSROUTER_CORE_DllAPI WildcardTopic(
            const std::string& topic_name,
            bool has_keyed_set = false,
            bool topic_with_key = false) noexcept;

    // TODO: extend test and documentation to admit ? and []

    //! Override \c contains method from \c FilterTopic
    DDSROUTER_CORE_DllAPI bool contains(
            const FilterTopic& other) const override;

    /**
     * Override \c matches method from \c FilterTopic
     *
     * It uses \c fnmatch function, so it also contemplates ? and [] apart from *
     */
    DDSROUTER_CORE_DllAPI bool matches(
            const RealTopic& other) const override;
};

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_WILDCARDTOPIC_HPP_ */