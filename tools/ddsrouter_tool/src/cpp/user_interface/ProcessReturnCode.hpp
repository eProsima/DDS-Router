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
 * @file ProcessReturnCode.hpp
 *
 */

#ifndef EPROSIMA_DDSROUTER_USERINTERFACE_PROCESSRETURNCODE_HPP
#define EPROSIMA_DDSROUTER_USERINTERFACE_PROCESSRETURNCODE_HPP

namespace eprosima {
namespace ddsrouter {
namespace ui {

enum class ProcessReturnCode : int
{
    success = 0,
    help_argument = 1,
    version_argument = 2,
    incorrect_argument = 10,
    required_argument_failed = 11,
    execution_failed = 20,
};

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* EPROSIMA_DDSROUTER_USERINTERFACE_PROCESSRETURNCODE_HPP */
