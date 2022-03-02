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

#ifndef _DDSROUTEREVENT_FILEWATCHERHANDLER_HPP_
#define _DDSROUTEREVENT_FILEWATCHERHANDLER_HPP_

#include <functional>
#include <string>

#include <FileWatch.hpp>

#include <ddsrouter_event/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

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
     * Calls \c unset_callback
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
    void start_filewatcher_nts_();

    /**
     * @brief Stop watching file
     *
     * @note Method called only from destructor
     */
    void stop_filewatcher_nts_();

    /**
     * @brief Override \c callback_set_ from \c EventHandler .
     *
     * It starts filewatcher if it has not been started.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_set_nts_() noexcept override;

    /**
     * @brief Override \c callback_set_ from \c EventHandler .
     *
     * It stops filewatcher if it has been started.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_unset_nts_() noexcept override;

    //! Path of file to watch
    std::string file_path_;

    //! File Watcher object
    std::unique_ptr<filewatch::FileWatch<std::string>> file_watch_handler_;

    //! Whether the file_watcher has already been started
    std::atomic<bool> filewatcher_started_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_FILEWATCHERHANDLER_HPP_ */
