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
 * @file FileWatcherHandler.hpp
 */

#ifndef _DDSROUTER_USERINTERFACE_FILEWATCHERHANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_FILEWATCHERHANDLER_HPP_

#include <functional>
#include <string>
#include <thread>

#include <FileWatch.hpp>

#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/user_interface/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

/**
 * TODO
 */
class FileWatcherHandler : public EventHandler<std::string>
{
public:

    FileWatcherHandler(
        std::string file_path,
        Duration_ms reload_time = 0);

    FileWatcherHandler(
        std::function<void(std::string)> callback,
        std::string file_path,
        Duration_ms reload_time = 0);

    ~FileWatcherHandler();

protected:

    void start_filewatcher_();

    void stop_filewatcher_();

    void reload_thread_routine_();

    void start_reload_thread_();

    void stop_reload_thread_();

    std::string file_path_;

    Duration_ms reload_time_;

    std::unique_ptr<filewatch::FileWatch<std::string>> file_watch_handler_;

    std::atomic<bool> filewatcher_set_;

    std::atomic<bool> reload_set_;

    std::thread reload_thread_;
};

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_USERINTERFACE_FILEWATCHERHANDLER_HPP_ */
