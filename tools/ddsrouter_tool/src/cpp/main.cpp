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

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_event/FileWatcherHandler.hpp>
#include <ddsrouter_event/MultipleEventHandler.hpp>
#include <ddsrouter_event/PeriodicEventHandler.hpp>
#include <ddsrouter_event/SignalHandler.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>
#include <ddsrouter_utils/Time.hpp>
#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>
#include <ddsrouter_yaml/YamlManager.hpp>

#include "user_interface/constants.hpp"
#include "user_interface/arguments_configuration.hpp"
#include "user_interface/ProcessReturnCode.hpp"

using namespace eprosima::ddsrouter;

int main(
        int argc,
        char** argv)
{
    logUser(DDSROUTER_EXECUTION, "Starting DDS Router execution.");

    // Configuration File path
    std::string file_path = ui::DEFAULT_CONFIGURATION_FILE_NAME;

    // Reload time
    utils::Duration_ms reload_time = 0;

    // Debug option active
    bool activate_debug = false;

    // Parse arguments
    ui::ProcessReturnCode arg_parse_result =
            ui::parse_arguments(argc, argv, file_path, reload_time, activate_debug);

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
        utils::Log::SetVerbosity(utils::Log::Kind::Info);

        // It will not filter any log, so Fast DDS logs will be visible unless Fast DDS is compiled
        // in non debug or with LOG_NO_INFO=ON.
        // This is the easiest way to allow to see Warnings and Errors from Fast DDS.
        // Change it when Log Module is independent and with more extensive API.
        // Log::SetCategoryFilter(std::regex("(DDSROUTER)"));
    }

    // Encapsulating execution in block to erase all memory correctly before closing process
    try
    {
        // First of all, block signals SIGINT and SIGTERM in this thread so they are only handled in dedicated threads
        sigset_t set;
        sigemptyset(&set);
        sigaddset(&set, SIGINT);
        sigaddset(&set, SIGTERM);
        sigprocmask(SIG_BLOCK, &set, NULL);

        // Create signal handlers
        event::MultipleEventHandler signal_handlers;

        signal_handlers.register_event_handler<event::EventHandler<int>, int>(
            std::make_unique<event::SignalHandler<event::SIGNAL_SIGINT>>());     // Add SIGINT
        signal_handlers.register_event_handler<event::EventHandler<int>, int>(
            std::make_unique<event::SignalHandler<event::SIGNAL_SIGTERM>>());    // Add SIGTERM

        /////
        // DDS Router Initialization

        // Load DDS Router Configuration
        core::configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file(file_path);

        // Create DDS Router
        core::DDSRouter router(router_configuration);

        /////
        // File Watcher Handler

        // Callback will reload configuration and pass it to DDSRouter
        // WARNING: it is needed to pass file_path, as FileWatcher only retrieves file_name
        std::function<void(std::string)> filewatcher_callback =
                [&router, file_path]
                    (std::string file_name)
                {
                    logUser(DDSROUTER_EXECUTION, "FileWatcher event raised. Reloading configuration.");
                    try
                    {
                        core::configuration::DDSRouterConfiguration router_configuration =
                                yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file(file_path);
                        router.reload_configuration(router_configuration);
                    }
                    catch (const std::exception& e)
                    {
                        logWarning(DDSROUTER_EXECUTION,
                                "Error reloading configuration file " << file_name << " with error: " << e.what());
                    }
                };

        // Creating FileWatcher event handler
        std::unique_ptr<event::FileWatcherHandler> file_watcher_handler =
                std::make_unique<event::FileWatcherHandler>(filewatcher_callback, file_path);

        /////
        // Periodic Handler for reload configuration in periodic time

        // It must be a ptr, so the object is only created when required by a specific configuration
        std::unique_ptr<event::PeriodicEventHandler> periodic_handler;

        // If reload time is higher than 0, create a periodic event to reload configuration
        if (reload_time > 0)
        {
            // Callback will reload configuration and pass it to DDSRouter
            std::function<void()> periodic_callback =
                    [&router, file_path]
                        ()
                    {
                        logUser(DDSROUTER_EXECUTION, "Periodic event raised. Reloading configuration.");
                        try
                        {
                            core::configuration::DDSRouterConfiguration router_configuration =
                                    yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file(file_path);
                            router.reload_configuration(router_configuration);
                        }
                        catch (const std::exception& e)
                        {
                            logWarning(DDSROUTER_EXECUTION,
                                    "Error reloading configuration file " << file_path << " with error: " << e.what());
                        }
                    };

            periodic_handler = std::make_unique<event::PeriodicEventHandler>(periodic_callback, reload_time);
        }

        // Start Router
        router.start();

        // Wait until signal arrives
        signal_handlers.wait_for_event();

        // Before stopping the Router erase event handlers that reload configuration
        if (periodic_handler)
        {
            periodic_handler.reset();
        }

        if (file_watcher_handler)
        {
            file_watcher_handler.reset();
        }

        // Stop Router
        router.stop();
    }
    catch (const utils::ConfigurationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Loading DDS Router Configuration from file " << file_path <<
                ". Error message:\n " <<
                e.what());
        return ui::ProcessReturnCode::EXECUTION_FAILED;
    }
    catch (const utils::InitializationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Initializing DDS Router. Error message:\n " <<
                e.what());
        return ui::ProcessReturnCode::EXECUTION_FAILED;
    }

    logUser(DDSROUTER_EXECUTION, "Finishing DDS Router execution correctly.");

    // Force print every log before closing
    utils::Log::Flush();

    return ui::ProcessReturnCode::SUCCESS;
}
