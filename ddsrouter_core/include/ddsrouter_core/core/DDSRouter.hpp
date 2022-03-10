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
 * @file DDSRouter.hpp
 */

#ifndef _DDSROUTERCORE_CORE_DDSROUTERCORE_HPP_
#define _DDSROUTERCORE_CORE_DDSROUTERCORE_HPP_

#include <memory>

#include <ddsrouter_utils/ReturnCode.hpp>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>


namespace eprosima {
namespace ddsrouter {
namespace core {

class DDSRouterImpl;

/**
 * TODO
 */
class DDSRouter
{
public:

    /**
     * @brief Construct a new DDSRouter object
     *
     * Initialize a whole DDSRouter:
     * - Create its associated AllowedTopicList
     * - Create Participants and add them to \c ParticipantsDatabase
     * - Create the Bridges for RealTopics as disabled (TODO: remove when discovery is ready)
     *
     * @param [in] configuration : Configuration for the new DDS Router
     *
     * @throw \c ConfigurationException in case the yaml inside allowlist is not well-formed
     * @throw \c InitializationException in case \c IParticipants , \c IWriters or \c IReaders creation fails.
     */
    DDSROUTER_CORE_DllAPI DDSRouter(
            const configuration::DDSRouterConfiguration& configuration);

    /**
     * @brief Destroy the DDSRouter object
     *
     * Stop the DDSRouter
     * Destroy all Bridges
     * Destroy all Participants
     */
    DDSROUTER_CORE_DllAPI virtual ~DDSRouter();

    // EVENTS
    /**
     * @brief Reload the allowed topic configuration
     *
     * @param [in] configuration : new configuration
     *
     * @return \c RETCODE_OK if configuration has been updated correctly
     * @return \c RETCODE_NO_DATA if new configuration has not changed
     * @return \c RETCODE_ERROR if any other error has occurred
     *
     * @throw \c ConfigurationException in case the new yaml is not well-formed
     */
    DDSROUTER_CORE_DllAPI utils::ReturnCode reload_configuration(
            const configuration::DDSRouterReloadConfiguration& configuration);

    /**
     * @brief Start communication in DDS Router
     *
     * Enable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    DDSROUTER_CORE_DllAPI utils::ReturnCode start() noexcept;

    /**
     * @brief Stop communication in DDS Router
     *
     * Disable every topic Bridge.
     *
     * @note this method returns a ReturnCode for future possible errors
     *
     * @return \c RETCODE_OK always
     */
    DDSROUTER_CORE_DllAPI utils::ReturnCode stop() noexcept;

protected:

    std::unique_ptr<DDSRouterImpl> ddsrouter_impl_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CORE_DDSROUTERCORE_HPP_ */
