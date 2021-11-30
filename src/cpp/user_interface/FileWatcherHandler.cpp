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

#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/user_interface/FileWatcherHandler.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace interface {

FileWatcherHandler::FileWatcherHandler(
        std::string file_path,
        Duration_ms reload_time /*= 0*/)
    : EventHandler<std::string>()
    , file_path_(file_path)
    , reload_time_(reload_time)
{
    start_filewatcher_();
    start_reload_thread_();
}

FileWatcherHandler::FileWatcherHandler(
        std::function<void(std::string)> callback,
        std::string file_path,
        Duration_ms reload_time /*= 0*/)
    : EventHandler<std::string>(callback)
    , file_path_(file_path)
{
    start_filewatcher_();
    start_reload_thread_();
}

FileWatcherHandler::~FileWatcherHandler()
{
    stop_reload_thread_();
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
                        logInfo(DDSROUTER_FILEWATCHER, "File: " << path << " modified. Reloading.");
                        call_callback_(path);
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
    filewatcher_set_.store(true);
}

void FileWatcherHandler::stop_filewatcher_()
{
    filewatcher_set_.store(false);
    file_watch_handler_.reset();
}

void FileWatcherHandler::reload_thread_routine_()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(reload_time_ * 1000));
        std::set<std::string> new_topics;
        std::set<std::string> old_topics;

        call_callback_(file_path_);
    }
}

void FileWatcherHandler::start_reload_thread_()
{
    if (reload_time_ > 0)
    {
        reload_thread_ = std::thread(
                    &FileWatcherHandler::reload_thread_routine_, this);
        reload_set_.store(true);
    }
}

void FileWatcherHandler::stop_reload_thread_()
{
    reload_set_.store(false);
    if (reload_thread_.joinable())
    {
        reload_thread_.detach();
    }
}

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */
