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
 * @file UserInterfaceManager.cpp
 *
 */

#include <ddsrouter/user_interface/UserInterfaceManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace interface {

UserInterfaceManager::UserInterfaceManager(
        std::unique_ptr<FileWatcherHandler> file_watch_handler,
        std::unique_ptr<SignalHandler> signal_handler)
    : file_watch_handler_(std::move(file_watch_handler))
    , signal_handler_(std::move(signal_handler))
{
    // TODO
}

ReturnCode UserInterfaceManager::main_routine()
{
    // TODO
    return ReturnCode::RETCODE_OK;
}

void UserInterfaceManager::stop(int signum)
{
    // TODO
}

void UserInterfaceManager::reload_configuration(const std::string& file_path)
{
    // TODO
}

RawConfiguration UserInterfaceManager::load_configuration_()
{
    // TODO
    return RawConfiguration();
}

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */
