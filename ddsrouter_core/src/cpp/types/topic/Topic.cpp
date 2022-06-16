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
 * @file Topic.cpp
 *
 */

#include <types/topic/Topic.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <array>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

static constexpr std::array<const char*, 1> REAL_TOPIC_INVALID_SUBSTRINGS = {
    "*", // Wildcard char
};

Topic::Topic(
        std::string name,
        std::string type,
        bool with_key /*false*/)
    : name_(name)
    , type_(type)
    , with_key_(with_key)
{
}

const std::string& Topic::name() const noexcept
{
    return name_;
}

const std::string& Topic::type() const noexcept
{
    return type_;
}

bool Topic::has_key() const noexcept
{
    return with_key_;
}

bool Topic::operator ==(
        const Topic& other) const noexcept
{
    return name_ == other.name() && type_ == other.type();
}

bool Topic::operator <(
        const Topic& other) const
{
    return std::make_pair(name_, type_) < std::make_pair(other.name(), other.type());
}

//// RealTopic

RealTopic::RealTopic(
        std::string name,
        std::string type,
        bool with_key,
        bool is_reliable)
    : Topic(name, type, with_key)
    , reliable_(is_reliable)
{
    for (auto invalid_substring : REAL_TOPIC_INVALID_SUBSTRINGS)
    {
        if (name_.find(invalid_substring) != std::string::npos ||
                type_.find(invalid_substring) != std::string::npos)
        {
            throw utils::InitializationException( utils::Formatter() << "Found invalid substring: " <<
                          invalid_substring);
        }
    }
}

bool RealTopic::is_reliable() const noexcept
{
    return reliable_;
}

//// FilterTopic

FilterTopic::FilterTopic(
        std::string name,
        std::string type,
        bool with_key,
        bool has_keyed_set)
    : Topic(name, type, with_key)
    , has_keyed_set_(has_keyed_set)
{
}

bool FilterTopic::has_keyed_set() const noexcept
{
    return has_keyed_set_;
}

bool FilterTopic::matches(
        const RealTopic& other) const noexcept
{

    return (!this->has_keyed_set() || (this->has_key() == other.has_key())) &&
           utils::match_pattern(this->name(), other.name()) &&
           utils::match_pattern(this->type(), other.type());
}

WildcardTopic::WildcardTopic(
        std::string name,
        std::string type, /* "*" */
        bool topic_with_key /* = false */,
        bool has_keyed_set /* = false */)
    : FilterTopic(name, type, topic_with_key, has_keyed_set)
{
}

bool WildcardTopic::contains(
        const FilterTopic& other) const
{
    // TODO: implement
    static_cast<void> (other);
    return false;
}

std::ostream& operator <<(
        std::ostream& os,
        const Topic& topic)
{
    os << "Topic{" << topic.name() << ";" << topic.type() << "}";
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const RealTopic& topic)
{
    os << "Topic{" << topic.name() << ";" << topic.type() << ";" << (topic.has_key()? "keyed" : "no_key") << ";" <<
        (topic.is_reliable()? "reliable" : "best_effort") << "}";
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const FilterTopic& topic)
{
    os << "FilterTopic(" << topic.name() << ";" << topic.type() << ";" << (topic.has_key()? "keyed" : "no_key") <<
        ";" << (topic.has_keyed_set()? "keyed_set" : "no_keyed_set") << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
