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

#include <FileWatch.hpp>

#include <ddsrouter/user_interface/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

/**
 * It implements the functionality to watch over a specific file and raise a callback
 * every time the file has changed.
 *
 * @warning Because of the FileWatcher implementation, each callback is called twice.
 */
class FileWatcherHandler : public EventHandler<std::string>
{
public:

    /**
     * @brief Construct a new File Watcher for file \c file_path
     *
     * If callback is not set, FileWatcher will not warn of updates in document.
     *
     * @param file_path : path for the file to watch
     */
    FileWatcherHandler(
            std::string file_path);


    /**
     * @brief Construct an FileWatcherHandler with a specific callback.
     *
     * @param file_path : path for the file to watch
     * @param callback : function that will be called when the event raises.
     */
    FileWatcherHandler(
            std::function<void(std::string)> callback,
            std::string file_path);

    /**
     * @brief Destroy the File Watcher Handler object
     *
     * Stop file watching
     */
    ~FileWatcherHandler();

protected:

    /**
     * @brief Start watching file
     *
     * It uses external library \c filewatch to create a File Watcher
     *
     * @note Method called only from constructor
     */
    void start_filewatcher_();

    /**
     * @brief Stop watching file
     *
     * @note Method called only from destructor
     */
    void stop_filewatcher_();

    //! Path of file to watch
    std::string file_path_;

    //! File Watcher object
    std::unique_ptr<filewatch::FileWatch<std::string>> file_watch_handler_;
};

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_USERINTERFACE_FILEWATCHERHANDLER_HPP_ */
