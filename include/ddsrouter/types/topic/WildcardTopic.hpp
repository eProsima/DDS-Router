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

#ifndef _DDSROUTER_TYPES_TOPIC_WILDCARDTOPIC_HPP_
#define _DDSROUTER_TYPES_TOPIC_WILDCARDTOPIC_HPP_

#include <ddsrouter/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Concrete class that represents an FilterTopic that uses the special char "*" as any substring
 */
class WildcardTopic : public FilterTopic
{
public:

    //! Using parent constructos
    using FilterTopic::FilterTopic;

    //! Constructor that allows any type
    WildcardTopic(
            const std::string& topic_name);

    //! Destructor
    virtual ~WildcardTopic();

    // TODO: extend test and documentation to admit ? and []

    //! Override \c contains method from \c FilterTopic
    bool contains(
            const FilterTopic& other) const override;

    /**
     * Override \c matches method from \c FilterTopic
     *
     * It uses \c fnmatch function, so it also contamplates ? and [] apart from *
     */
    bool matches(
            const RealTopic& other) const override;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_TOPIC_WILDCARDTOPIC_HPP_ */
