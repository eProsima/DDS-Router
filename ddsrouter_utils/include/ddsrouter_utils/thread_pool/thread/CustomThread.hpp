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
 * @file CustomThread.hpp
 *
 * This file contains class CustomThread definition.
 */

#ifndef _DDSROUTERTHREAD__SRC_CPP_THREAD_CUSTOMTHREAD_HPP_
#define _DDSROUTERTHREAD__SRC_CPP_THREAD_CUSTOMTHREAD_HPP_

#include <thread>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * This class represents a thread that can be executed by a Thread Pool.
 *
 * @note this first implementation only uses this class as a \c std::thread for simplicity.
 * In future implementations, this could be a more complex class.
 */
class CustomThread : public std::thread
{
    using std::thread::thread;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD__SRC_CPP_THREAD_CUSTOMTHREAD_HPP_ */
