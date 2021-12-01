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
 * @file main.cpp
 *
 */

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/utils.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/constants.hpp>

using namespace eprosima::ddsrouter;

int main(
        int argc,
        char** argv)
{
    // TODO: main

    static_cast<void>(argc);
    static_cast<void>(argv);

    // Activate log
    Log::SetVerbosity(Log::Kind::Info);
    Log::SetCategoryFilter(std::regex("(DDSROUTER)"));

    // Generate configuration
    std::string file_name = DEFAULT_CONFIGURATION_FILE_NAME;
    RawConfiguration configuration = load_configuration_from_file(file_name);

    // Create DDSRouter entity
    DDSRouter router(configuration);

    // Start DDSRouter
    router.start();

    std::this_thread::sleep_for(std::chrono::seconds(60));

    // Stop DDS Router
    router.stop();

    return 0;
}
