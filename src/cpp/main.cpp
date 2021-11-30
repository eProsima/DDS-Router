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

#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/types/constants.hpp>
#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/user_interface/FileWatcherHandler.hpp>
#include <ddsrouter/user_interface/SignalHandler.hpp>

using namespace eprosima::ddsrouter;


int main(
        int argc,
        char** argv)
{
    std::cout << "Starting DDS Router execution." << std::endl;

    // TODO do it depending on arguments

    // Activate log
    Log::SetVerbosity(Log::Kind::Info);
    Log::SetCategoryFilter(std::regex("(DDSROUTER)"));

    // Encapsulating execution in block to erase all memory correctly before closing process
    {
        // Configuration File path
        std::string file_path = DEFAULT_CONFIGURATION_FILE_NAME;

        // Reload time
        bool is_reload_set = false;
        eprosima::ddsrouter::Duration_ms reload_time;

        // TODO parse arguments


        // Load DDS Router Configuration
        RawConfiguration router_configuration = load_configuration_from_file(file_path);

        // Create DDS Router
        DDSRouter router(router_configuration);

        // File Watcher Handler
        // Callback will reload configuration and pass it to DDSRouter
        std::function<void(std::string)> filewatcher_callback =
            [&router]
            (std::string file_path)
            {
                std::cout << "Reloading configuration." << std::endl;
                try
                {
                    RawConfiguration router_configuration = load_configuration_from_file(file_path);
                    router.reload_configuration(router_configuration);
                }
                catch(const std::exception& e)
                {
                    std::cerr <<
                        "Error reloading configuration file " << file_path << " with error: " << e.what() <<
                        std::endl;
                }
            };

        // Creating FileWatcher event handler
        interface::FileWatcherHandler file_watcher_handler(filewatcher_callback, file_path, reload_time);

        // Signal handler
        // std::unique_ptr<interface::SignalHandler<interface::Signals::SIGNAL_SIGINT>> signal_handler();
        interface::SignalHandler<interface::SIGNAL_SIGINT> signal_handler;

        // Wait until signal arrives
        signal_handler.wait_for_event();

        // Stopping DDS Router
        router.stop();
    }

    std::cout << "Finishing DDS Router execution correctly." << std::endl;

    return EXIT_SUCCESS;
}
