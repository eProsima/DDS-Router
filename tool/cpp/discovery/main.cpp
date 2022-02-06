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
 * @file main.cpp
 *
 */

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/event/FileWatcherHandler.hpp>
#include <ddsrouter/event/PeriodicEventHandler.hpp>
#include <ddsrouter/event/SignalHandler.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/constants.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter/yaml/YamlManager.hpp>

#include "user_interface/arguments_configuration.hpp"
#include "user_interface/ProcessReturnCode.hpp"
#include "yaml/YamlReaderDiscoveryConfiguration.hpp"

using namespace eprosima::ddsrouter::discovery;

int main(
        int argc,
        char** argv)
{
    logUser(DDSROUTER_DISCOVERYTOOL, "Starting DiscoveryTool execution.");

    // Configuration File path
    std::string file_path = ui::DEFAULT_CONFIGURATION_FILE_NAME;

    // Debug option active
    bool activate_debug = false;

    // Parse arguments
    ui::ProcessReturnCode arg_parse_result =
            ui::parse_arguments(argc, argv, file_path, activate_debug);

    if (arg_parse_result == ui::ProcessReturnCode::HELP_ARGUMENT)
    {
        return ui::ProcessReturnCode::SUCCESS;
    }
    else if (arg_parse_result != ui::ProcessReturnCode::SUCCESS)
    {
        return arg_parse_result;
    }

    // Activate Debug
    if (activate_debug)
    {
        // Activate log
        eprosima::ddsrouter::Log::SetVerbosity(eprosima::ddsrouter::Log::Kind::Info);

        // It will not filter any log, so Fast DDS logs will be visible unless Fast DDS is compiled
        // in non debug or with LOG_NO_INFO=ON.
        // This is the easiest way to allow to see Warnings and Errors from Fast DDS.
        // Change it when Log Module is independent and with more extensive API.
        // Log::SetCategoryFilter(std::regex("(DDSROUTER)"));
    }

    // Encapsulating execution in block to erase all memory correctly before closing process
    try
    {
        // First of all, create signal handler so SIGINT does not break the program while initializing
        // Signal handler
        std::unique_ptr<
            eprosima::ddsrouter::event::SignalHandler<eprosima::ddsrouter::event::SIGNAL_SIGINT>>
                signal_handler =
                    std::make_unique<
                        eprosima::ddsrouter::event::SignalHandler<eprosima::ddsrouter::event::SIGNAL_SIGINT>> ();

        /////
        // DDS Router Initialization

        // Load Discovery Configuration
        eprosima::ddsrouter::configuration::DDSRouterConfiguration router_configuration =
            yaml::YamlReaderDiscoveryConfiguration::load_discovery_configuration_from_file(file_path);

        eprosima::ddsrouter::utils::Formatter error_msg;
        if (!router_configuration.is_valid(error_msg))
        {
            logError(DDSROUTER_DISCOVERYTOOL,
                "Incorrect DiscoveryTool Configuration. Error message:\n " <<
                    error_msg);
            return ui::ProcessReturnCode::CONFIGURATION_FAILED;
        }

        // Create DDS Router
        eprosima::ddsrouter::DDSRouter router(router_configuration);

        // Start Router
        router.start();

        // Wait until signal arrives
        signal_handler->wait_for_event();

        // Stop Router
        router.stop();
    }
    catch (const eprosima::ddsrouter::ConfigurationException& e)
    {
        logError(DDSROUTER_DISCOVERYTOOL,
                "Error Loading DiscoveryTool Configuration from file " << file_path <<
                ". Error message:\n " <<
                e.what());
        return ui::ProcessReturnCode::CONFIGURATION_FAILED;
    }
    catch (const eprosima::ddsrouter::InitializationException& e)
    {
        logError(DDSROUTER_DISCOVERYTOOL,
                "Error Initializing DiscoveryTool. Error message:\n " <<
                e.what());
        return ui::ProcessReturnCode::INITIALIZATION_FAILED;
    }

    logUser(DDSROUTER_DISCOVERYTOOL, "Finishing DiscoveryTool execution correctly.");

    return ui::ProcessReturnCode::SUCCESS;
}
