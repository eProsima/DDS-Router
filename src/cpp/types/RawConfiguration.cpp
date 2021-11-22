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
 * @file RawConfiguration.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

RawConfiguration load_configuration_from_file(const std::string& file_path)
{
    try
    {
        return YAML::LoadFile(file_path);
    }
    catch (const std::exception& e)
    {
        throw ConfigurationException(utils::Formatter() << "Error occured while loading configuration from file: "
            << file_path << " : " << e.what());
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
