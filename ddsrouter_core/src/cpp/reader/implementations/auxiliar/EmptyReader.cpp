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
 * @file EmptyReader.cpp
 */

#include <reader/implementations/auxiliar/EmptyReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

void EmptyReader::enable() noexcept
{
}

void EmptyReader::disable() noexcept
{
}

void EmptyReader::set_on_data_available_callback(
        std::function<void()>) noexcept
{
}

void EmptyReader::unset_on_data_available_callback() noexcept
{
}

utils::ReturnCode EmptyReader::take(
        std::unique_ptr<DataReceived>&) noexcept
{
    return utils::ReturnCode::RETCODE_NO_DATA;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
