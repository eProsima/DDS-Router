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
 * @file DDSRouterReloadConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_

#include <memory>
#include <set>

#include <ddsrouter/configuration/BaseConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
class DDSRouterReloadConfiguration : public BaseConfiguration
{
public:

    /**
     * TODO
     */
    DDSRouterReloadConfiguration(
            std::set<std::shared_ptr<FilterTopic>> allowlist,
            std::set<std::shared_ptr<FilterTopic>> blocklist,
            std::set<std::shared_ptr<RealTopic>> builtin_topics);

    /**
     * @brief Return a set with the topics allowed in the configuration
     *
     * @return Set of filters to get allowed topics
     */
    std::set<std::shared_ptr<FilterTopic>> allowlist() const noexcept;

    /**
     * @brief Return a set with the topics blocked in the configuration
     *
     * @return Set of filters to get blocked topics
     */
    std::set<std::shared_ptr<FilterTopic>> blocklist() const noexcept;

    /**
     * TODO
     */
    std::set<std::shared_ptr<RealTopic>> builtin_topics() const noexcept;

    bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

protected:

    std::set<std::shared_ptr<FilterTopic>> allowlist_;
    std::set<std::shared_ptr<FilterTopic>> blocklist_;
    std::set<std::shared_ptr<RealTopic>> builtin_topics_;
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_ */
