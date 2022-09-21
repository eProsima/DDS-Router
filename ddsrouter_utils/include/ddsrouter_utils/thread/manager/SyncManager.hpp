// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file SyncManager.hpp
 *
 * This file contains class SyncManager definition.
 */

#pragma once

#include <map>
#include <thread>
#include <vector>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/thread/manager/IManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

/**
 * TODO
 */
class SyncManager : public IManager
{
public:
    // virtual void execute(const ITask& task) override;
    // virtual void execute(ITask&& task) override;
    virtual void execute(std::unique_ptr<ITask>&& task) override;
};

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
