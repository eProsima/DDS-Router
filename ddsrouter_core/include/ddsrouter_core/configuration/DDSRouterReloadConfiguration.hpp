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

#ifndef _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_

#include <memory>
#include <set>

#include <ddsrouter_utils/Formatter.hpp>

#include <ddsrouter_core/configuration/BaseConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/FilterTopic.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
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
    DDSROUTER_CORE_DllAPI DDSRouterReloadConfiguration(
            std::set<std::shared_ptr<types::FilterTopic>> allowlist,
            std::set<std::shared_ptr<types::FilterTopic>> blocklist,
            std::set<std::shared_ptr<types::RealTopic>> builtin_topics);

    /**
     * @brief Return a set with the topics allowed in the configuration
     *
     * @return Set of filters to get allowed topics
     */
    DDSROUTER_CORE_DllAPI std::set<std::shared_ptr<types::FilterTopic>> allowlist() const noexcept;

    /**
     * @brief Return a set with the topics blocked in the configuration
     *
     * @return Set of filters to get blocked topics
     */
    DDSROUTER_CORE_DllAPI std::set<std::shared_ptr<types::FilterTopic>> blocklist() const noexcept;

    /**
     * TODO
     */
    DDSROUTER_CORE_DllAPI std::set<std::shared_ptr<types::RealTopic>> builtin_topics() const noexcept;

    DDSROUTER_CORE_DllAPI bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

protected:

    std::set<std::shared_ptr<types::FilterTopic>> allowlist_;
    std::set<std::shared_ptr<types::FilterTopic>> blocklist_;
    std::set<std::shared_ptr<types::RealTopic>> builtin_topics_;
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_ */
