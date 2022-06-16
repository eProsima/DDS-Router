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

#ifndef _DDSROUTERCORE_TYPES_TOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_utils/exception/InitializationException.hpp>

#include <set>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Base topic class
 */
class Topic
{
public:

    /**
     * @brief Constructs a topic.
     *
     */
    DDSROUTER_CORE_DllAPI Topic(
            std::string name,
            std::string type,
            bool with_key = false);

    DDSROUTER_CORE_DllAPI Topic(
            const Topic&) = default;
    DDSROUTER_CORE_DllAPI Topic(
            Topic&&) = default;

    DDSROUTER_CORE_DllAPI Topic& operator =(
            const Topic&) = default;
    DDSROUTER_CORE_DllAPI Topic& operator =(
            Topic&&) = default;

    //! Field-wise comparison
    DDSROUTER_CORE_DllAPI bool operator ==(
            const Topic&) const noexcept;

    //! Lexicographic compare
    DDSROUTER_CORE_DllAPI bool operator <(
            const Topic&) const;

    //! Name getter
    DDSROUTER_CORE_DllAPI const std::string& name() const noexcept;

    //! Type getter
    DDSROUTER_CORE_DllAPI const std::string& type() const noexcept;

    //! Whether has key or not
    DDSROUTER_CORE_DllAPI bool has_key() const noexcept;

protected:

    //! Topic name
    std::string name_;

    //! Topic type
    std::string type_;

    //! Topic has key
    bool with_key_;
};

//! RealTopic class
class RealTopic : public Topic
{
private:

public:

    /**
     * @brief Constructs a RealTopic
     *
     */
    DDSROUTER_CORE_DllAPI RealTopic(
            std::string name,
            std::string type,
            bool with_key = false,
            bool is_reliable = false);

    //! Whether it is reliable
    DDSROUTER_CORE_DllAPI bool is_reliable() const noexcept;

protected:

    //! Topic is reliable or not
    bool reliable_;
};

//! FilterTopic class to that matches against RealTopics
class FilterTopic : public Topic
{
public:

    /**
     * @brief Constructs a FilterTopic
     *
     */
    DDSROUTER_CORE_DllAPI FilterTopic(
            std::string name,
            std::string type = "*",
            bool with_key = false,
            bool has_keyed_set = false);

    //! Whether has keyed set
    DDSROUTER_CORE_DllAPI bool has_keyed_set() const noexcept;


    /**
     * @brief Constructs client-and-server-oriented TLSConfiguration
     *
     * @param other Input topic to match
     */
    bool matches(
            const RealTopic& other) const noexcept;

protected:

    //! Topic has keyed set or not
    bool has_keyed_set_;
};

DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Topic& topic);

DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const RealTopic& topic);

DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const FilterTopic& topic);


//! Temporary alias of a FilterTopic, with a specific constructor.
// TODO: This is a candidate for deprecation, as it is not currently adding any specific feature.
class WildcardTopic : public FilterTopic
{
public:

    DDSROUTER_CORE_DllAPI WildcardTopic(
            std::string name,
            std::string type = "*",
            bool topic_with_key = false,
            bool has_keyed_set = false);

    DDSROUTER_CORE_DllAPI bool contains(
            const FilterTopic& other) const;
};

//! Set of topics, current implementation as std::set, but there is potential interest for having a thread-safe set container implemented in the future
template <typename SpecializedTopicT>
using TopicKeySet = std::set<SpecializedTopicT>;

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_HPP_ */
