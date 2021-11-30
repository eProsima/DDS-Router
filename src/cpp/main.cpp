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

#include <ddsrouter/user_interface/UserInterfaceManager.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/user_interface/FileWatcherHandler.hpp>
#include <ddsrouter/user_interface/SignalHandler.hpp>

using namespace eprosima::ddsrouter::interface;

int main(
        int argc,
        char** argv)
{
    // Configuration File path
    std::string file_path;

    // Reload time
    bool is_reload_set = false;
    eprosima::ddsrouter::Duration_ms reload_time;

    // TODO parse arguments

    // File Watcher Handler
    std::unique_ptr<FileWatcherHandler> file_watcher_handler;
    if(is_reload_set)
    {
        file_watcher_handler = std::make_unique<FileWatcherHandler>(file_path, reload_time);
    }
    else
    {
        file_watcher_handler = std::make_unique<FileWatcherHandler>(file_path);
    }

    // Signal handler
    std::unique_ptr<SignalHandler> signal_handler;
    signal_handler = std::make_unique<SignalHandler>();

    // User Interface Handler
    UserInterfaceManager user_interface(
        std::move(file_watcher_handler),
        std::move(signal_handler));

    // Return value
    eprosima::ddsrouter::ReturnCode return_state = user_interface.main_routine();

    // TODO parse return
    return return_state();
}
