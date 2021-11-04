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

#ifndef _DATABROKER_TYPES_TOPIC_WILDCARDTOPIC_HPP_
#define _DATABROKER_TYPES_TOPIC_WILDCARDTOPIC_HPP_

#include <databroker/types/topic/AbstractTopic.hpp>

namespace eprosima {
namespace databroker {

/**
 * Concrete class that represents an AbstractTopic that uses the special char "*" as any substring
 */
class WildcardTopic : public AbstractTopic
{
public:

    //! Using parent constructos
    using AbstractTopic::AbstractTopic;

    //! Destructor
    virtual ~WildcardTopic();

    // TODO: extend test and documentation to admit ? and []

    //! Override \c contains method from \c AbstractTopic
    bool contains(
            const AbstractTopic& other) const override;

    /**
     * Override \c matches method from \c AbstractTopic
     *
     * It uses \c fnmatch function, so it also contamplates ? and [] apart from *
     */
    bool matches(
            const RealTopic& other) const override;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_TOPIC_WILDCARDTOPIC_HPP_ */
