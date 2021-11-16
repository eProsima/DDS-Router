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
 * @file Configuration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_

#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class Configuration
{
public:

    Configuration(
            const RawConfiguration& raw_configuration);

    virtual ~Configuration();

    std::list<std::shared_ptr<FilterTopic>> allowlist() const;

    std::list<std::shared_ptr<FilterTopic>> blocklist() const;

    std::map<ParticipantId, RawConfiguration> participants_configurations() const;

    //! Ad hoc function to find real topics within the allowlist
    // TODO: This method will disappear once the dynamic module is implemented
    std::set<RealTopic> real_topics() const;

protected:

    std::list<std::shared_ptr<FilterTopic>> generic_get_topic_list_(
            const char* list_tag) const;

    const RawConfiguration raw_configuration_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DDS_ROUTERCONFIGURATION_HPP_ */
