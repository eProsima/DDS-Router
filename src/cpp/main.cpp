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

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/event/FileWatcherHandler.hpp>
#include <ddsrouter/event/MultipleEventHandler.hpp>
#include <ddsrouter/event/PeriodicEventHandler.hpp>
#include <ddsrouter/event/SignalHandler.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/constants.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/user_interface/arguments_configuration.hpp>
#include <ddsrouter/user_interface/ProcessReturnCode.hpp>

using namespace eprosima::ddsrouter;


int main(
        int argc,
        char** argv)
{
    logUser(DDSROUTER_EXECUTION, "Starting DDS Router execution.");

    // Configuration File path
    std::string file_path = DEFAULT_CONFIGURATION_FILE_NAME;

    // Reload time
    eprosima::ddsrouter::Duration_ms reload_time = 0;

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

        // It will not filter any log, so Fast DDS logs will be visible unless Fast DDS is compiled
        // in non debug or with LOG_NO_INFO=ON.
        // This is the easiest way to allow to see Warnings and Errors from Fast DDS.
        // Change it when Log Module is independent and with more extensive API.
        // Log::SetCategoryFilter(std::regex("(DDSROUTER)"));
    }

    // Encapsulating execution in block to erase all memory correctly before closing process
    try
    {
        // First of all, create signal handler so SIGINT and SIGTERM does not break the program while initializing
        event::MultipleEventHandler signal_handlers;

        signal_handlers.register_event_handler<event::EventHandler<int>, int>(
            std::make_unique<event::SignalHandler<event::SIGNAL_SIGINT>>());     // Add SIGINT
        signal_handlers.register_event_handler<event::EventHandler<int>, int>(
            std::make_unique<event::SignalHandler<event::SIGNAL_SIGTERM>>());    // Add SIGTERM

        /////
        // DDS Router Initialization

        // Load DDS Router Configuration
        RawConfiguration router_configuration = load_configuration_from_file(file_path);


        // Create DDS Router
        DDSRouter router(router_configuration);

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
                        RawConfiguration router_configuration = load_configuration_from_file(file_path);
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
                            RawConfiguration router_configuration = load_configuration_from_file(file_path);
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
    catch (const ConfigurationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Loading DDS Router Configuration from file " << file_path <<
                ". Error message: " << e.what());
        return ui::ProcessReturnCode::EXECUTION_FAILED;
    }
    catch (const InitializationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Initializing DDS Router. Error message: " << e.what());
        return ui::ProcessReturnCode::EXECUTION_FAILED;
    }

    logUser(DDSROUTER_EXECUTION, "Finishing DDS Router execution correctly.");

    return ui::ProcessReturnCode::SUCCESS;
}
