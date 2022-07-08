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
 * @file BlankReader.cpp
 */

#include <reader/implementations/auxiliar/BlankReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

void BlankReader::enable() noexcept
{
}

void BlankReader::disable() noexcept
{
}

void BlankReader::set_on_data_available_callback(
        std::function<void()>) noexcept
{
}

void BlankReader::unset_on_data_available_callback() noexcept
{
}

utils::ReturnCode BlankReader::take(
        fastrtps::rtps::CacheChange_t*& cache_change) noexcept
{
    return utils::ReturnCode::RETCODE_NO_DATA;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
