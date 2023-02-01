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
 * @file DDSRouter.cpp
 *
 */

#include <ddsrouter_core/core/DDSRouter.hpp>
#include <core/DDSRouterImpl.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

// TODO: Use initial topics to start execution and start bridges

DDSRouter::DDSRouter(
        const configuration::DDSRouterConfiguration& configuration,
        const std::shared_ptr<DiscoveryDatabase>& discovery_database,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        const std::shared_ptr<ParticipantsDatabase>& participants_database)
    : ddsrouter_impl_(std::make_unique<DDSRouterImpl>(
        configuration, discovery_database, payload_pool, participants_database))
{
}

DDSRouter::~DDSRouter()
{
}

utils::ReturnCode DDSRouter::reload_configuration(
        const configuration::DDSRouterReloadConfiguration& new_configuration)
{
    return ddsrouter_impl_->reload_configuration(new_configuration);
}

utils::ReturnCode DDSRouter::start() noexcept
{
    return ddsrouter_impl_->start();
}

utils::ReturnCode DDSRouter::stop() noexcept
{
    return ddsrouter_impl_->stop();
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
