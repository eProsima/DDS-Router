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

#include <ddsrouter/exception/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/event/FileWatcherHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

FileWatcherHandler::FileWatcherHandler(
        std::string file_path)
    : EventHandler<std::string>()
    , file_path_(file_path)
    , filewatcher_started_(false)
{
    logDebug(
        DDSROUTER_PERIODICHANDLER,
        "FileWatcher Event Handler created with file path " << file_path_ << " .");
}

FileWatcherHandler::FileWatcherHandler(
        std::function<void(std::string)> callback,
        std::string file_path)
    : FileWatcherHandler(file_path)
{
    set_callback(callback);
}

FileWatcherHandler::~FileWatcherHandler()
{
    unset_callback();
}

void FileWatcherHandler::start_filewatcher_nts_()
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

    filewatcher_started_.store(true);

    logInfo(DDSROUTER_FILEWATCHER, "Start Watching file: " << file_path_);
}

void FileWatcherHandler::stop_filewatcher_nts_()
{
    file_watch_handler_.reset();
    filewatcher_started_.store(false);

    logInfo(DDSROUTER_FILEWATCHER, "Stop Watching file: " << file_path_);
}

void FileWatcherHandler::callback_set_nts_() noexcept
{
    if (!filewatcher_started_)
    {
        start_filewatcher_nts_();
    }
}

void FileWatcherHandler::callback_unset_nts_() noexcept
{
    if (filewatcher_started_)
    {
        stop_filewatcher_nts_();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
