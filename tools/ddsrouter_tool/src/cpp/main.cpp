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

#include <cpp_utils/event/FileWatcherHandler.hpp>
#include <cpp_utils/event/MultipleEventHandler.hpp>
#include <cpp_utils/event/PeriodicEventHandler.hpp>
#include <cpp_utils/event/SignalEventHandler.hpp>
#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/logging/CustomStdLogConsumer.hpp>
#include <cpp_utils/ReturnCode.hpp>
#include <cpp_utils/time/time_utils.hpp>
#include <cpp_utils/utils.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/core/DdsRouter.hpp>

#include <ddsrouter_yaml/YamlReaderConfiguration.hpp>

#include "user_interface/constants.hpp"
#include "user_interface/arguments_configuration.hpp"
#include "user_interface/ProcessReturnCode.hpp"

using namespace eprosima::ddsrouter;

int main(
        int argc,
        char** argv)
{
    // Configuration File path
    std::string file_path = "";

    // Reload time
    eprosima::utils::Duration_ms reload_time = 0;

    // Maximum timeout
    eprosima::utils::Duration_ms timeout = 0;

    // Debug options
    std::string log_filter = "DDSROUTER";
    eprosima::fastdds::dds::Log::Kind log_verbosity = eprosima::fastdds::dds::Log::Kind::Warning;

    // Parse arguments
    ui::ProcessReturnCode arg_parse_result =
            ui::parse_arguments(argc, argv, file_path, reload_time, timeout, log_filter, log_verbosity);

    if (arg_parse_result == ui::ProcessReturnCode::help_argument)
    {
        return static_cast<int>(ui::ProcessReturnCode::success);
    }
    else if (arg_parse_result == ui::ProcessReturnCode::version_argument)
    {
        return static_cast<int>(ui::ProcessReturnCode::success);
    }
    else if (arg_parse_result != ui::ProcessReturnCode::success)
    {
        return static_cast<int>(arg_parse_result);
    }

    // Check file is in args, else get the default file
    if (file_path == "")
    {
        file_path = ui::DEFAULT_CONFIGURATION_FILE_NAME;

        logUser(
            DDSROUTER_EXECUTION,
            "Not configuration file given, using default file " << file_path << ".");
    }

    // Check file exists and it is readable
    // NOTE: this check is redundant with option parse arg check
    if (!is_file_accessible(file_path.c_str(), eprosima::utils::FileAccessMode::read))
    {
        logError(
            DDSROUTER_ARGS,
            "File '" << file_path << "' does not exist or it is not accessible.");
        return static_cast<int>(ui::ProcessReturnCode::required_argument_failed);
    }

    logUser(DDSROUTER_EXECUTION, "Starting DDS Router Tool execution.");

    // Debug
    {
        // Remove every consumer
        eprosima::utils::Log::ClearConsumers();

        // Activate log with verbosity, as this will avoid running log thread with not desired kind
        eprosima::utils::Log::SetVerbosity(log_verbosity);

        eprosima::utils::Log::RegisterConsumer(
            std::make_unique<eprosima::utils::CustomStdLogConsumer>(log_filter, log_verbosity));

        // NOTE:
        // It will not filter any log, so Fast DDS logs will be visible unless Fast DDS is compiled
        // in non debug or with LOG_NO_INFO=ON.
        // This is the easiest way to allow to see Warnings and Errors from Fast DDS.
        // Change it when Log Module is independent and with more extensive API.
        // eprosima::utils::Log::SetCategoryFilter(std::regex("(DDSROUTER)"));
    }

    // Encapsulating execution in block to erase all memory correctly before closing process
    try
    {
        // Create a multiple event handler that handles all events that make the router stop
        eprosima::utils::event::MultipleEventHandler close_handler;

        // First of all, create signal handler so SIGINT and SIGTERM do not break the program while initializing
        close_handler.register_event_handler<eprosima::utils::event::EventHandler<eprosima::utils::event::Signal>,
                eprosima::utils::event::Signal>(
            std::make_unique<eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint>>());     // Add SIGINT
        close_handler.register_event_handler<eprosima::utils::event::EventHandler<eprosima::utils::event::Signal>,
                eprosima::utils::event::Signal>(
            std::make_unique<eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigterm>>());    // Add SIGTERM

        // If it must be a maximum time, register a periodic handler to finish handlers
        if (timeout > 0)
        {
            close_handler.register_event_handler<eprosima::utils::event::PeriodicEventHandler>(
                std::make_unique<eprosima::utils::event::PeriodicEventHandler>(
                    []()
                    {
                        /* Do nothing */ },
                    timeout));
        }

        /////
        // DDS Router Initialization

        // Load DDS Router Configuration
        core::DdsRouterConfiguration router_configuration =
                yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file(file_path);

        // Create DDS Router
        core::DdsRouter router(router_configuration);

        /////
        // File Watcher Handler

        // Callback will reload configuration and pass it to DdsRouter
        // WARNING: it is needed to pass file_path, as FileWatcher only retrieves file_name
        std::function<void(std::string)> filewatcher_callback =
                [&router, file_path]
                    (std::string file_name)
                {
                    logUser(
                        DDSROUTER_EXECUTION,
                        "FileWatcher notified changes in file " << file_name << ". Reloading configuration");

                    try
                    {
                        core::DdsRouterConfiguration router_configuration =
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
        std::unique_ptr<eprosima::utils::event::FileWatcherHandler> file_watcher_handler =
                std::make_unique<eprosima::utils::event::FileWatcherHandler>(filewatcher_callback, file_path);

        /////
        // Periodic Handler for reload configuration in periodic time

        // It must be a ptr, so the object is only created when required by a specific configuration
        std::unique_ptr<eprosima::utils::event::PeriodicEventHandler> periodic_handler;

        // If reload time is higher than 0, create a periodic event to reload configuration
        if (reload_time > 0)
        {
            // Callback will reload configuration and pass it to DdsRouter
            std::function<void()> periodic_callback =
                    [&router, file_path]
                        ()
                    {
                        logUser(
                            DDSROUTER_EXECUTION,
                            "Periodic Timer raised. Reloading configuration from file " << file_path << ".");

                        try
                        {
                            core::DdsRouterConfiguration router_configuration =
                                    yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file(file_path);
                            router.reload_configuration(router_configuration);
                        }
                        catch (const std::exception& e)
                        {
                            logWarning(DDSROUTER_EXECUTION,
                                    "Error reloading configuration file " << file_path << " with error: " << e.what());
                        }
                    };

            periodic_handler = std::make_unique<eprosima::utils::event::PeriodicEventHandler>(periodic_callback,
                            reload_time);
        }

        // Start Router
        router.start();

        logUser(DDSROUTER_EXECUTION, "DDS Router running.");

        // Wait until signal arrives
        close_handler.wait_for_event();

        logUser(DDSROUTER_EXECUTION, "Stopping DDS Router.");

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

        logUser(DDSROUTER_EXECUTION, "DDS Router stopped correctly.");
    }
    catch (const eprosima::utils::ConfigurationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Loading DDS Router Configuration from file " << file_path <<
                ". Error message:\n " <<
                e.what());
        return static_cast<int>(ui::ProcessReturnCode::execution_failed);
    }
    catch (const eprosima::utils::InitializationException& e)
    {
        logError(DDSROUTER_ERROR,
                "Error Initializing DDS Router. Error message:\n " <<
                e.what());
        return static_cast<int>(ui::ProcessReturnCode::execution_failed);
    }

    logUser(DDSROUTER_EXECUTION, "Finishing DDS Router Tool execution correctly.");

    // Force print every log before closing
    eprosima::utils::Log::Flush();

    return static_cast<int>(ui::ProcessReturnCode::success);
}
