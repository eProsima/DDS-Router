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
 * @file UserInterfaceManager.hpp
 */

#ifndef _DDSROUTER_USERINTERFACE_USERINTERFACEHANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_USERINTERFACEHANDLER_HPP_

#include <string>

#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/user_interface/FileWatcherHandler.hpp>
#include <ddsrouter/user_interface/SignalHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace interface {

/**
 * TODO
 */
class UserInterfaceManager
{
public:

    UserInterfaceManager(
        std::unique_ptr<FileWatcherHandler> file_watch_handler,
        std::unique_ptr<SignalHandler> signal_handler);

    ReturnCode main_routine();

    void stop(int signum);

    void reload_configuration(const std::string& file_path);

protected:

    RawConfiguration load_configuration_();

    std::unique_ptr<FileWatcherHandler> file_watch_handler_;

    std::unique_ptr<SignalHandler> signal_handler_;

};

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_USERINTERFACE_USERINTERFACEHANDLER_HPP_ */
