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
 * @file FileWatcherHandler.cpp
 *
 */

#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/user_interface/FileWatcherHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

FileWatcherHandler::FileWatcherHandler(
        std::string file_path)
    : EventHandler<std::string>()
    , file_path_(file_path)
{
    start_filewatcher_();
}

FileWatcherHandler::FileWatcherHandler(
        std::function<void(std::string)> callback,
        std::string file_path)
    : EventHandler<std::string>(callback)
    , file_path_(file_path)
{
    start_filewatcher_();
}

FileWatcherHandler::~FileWatcherHandler()
{
    stop_filewatcher_();
}

void FileWatcherHandler::start_filewatcher_()
{
    logInfo(DDSROUTER_FILEWATCHER, "Starting FileWatcher in file: " << file_path_);

    try
    {
        file_watch_handler_ = std::make_unique<filewatch::FileWatch<std::string>>(
            file_path_,
            [this](const std::string& path, const filewatch::Event change_type)
            {
                switch (change_type)
                {
                    case filewatch::Event::modified:
                        logInfo(DDSROUTER_FILEWATCHER, "File: " << path << " modified.");
                        event_occurred_(path);
                        break;
                    default:
                        // No-op
                        break;
                }
            });
    }
    catch (const std::exception& e)
    {
        InitializationException(utils::Formatter() <<
            "Error creating file watcher: " << e.what());
    }

    logInfo(DDSROUTER_FILEWATCHER, "Watching file: " << file_path_);
}

void FileWatcherHandler::stop_filewatcher_()
{
    file_watch_handler_.reset();
}

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */
