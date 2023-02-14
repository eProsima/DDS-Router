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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

// In order to not create an enormous terrible file, each unit test has been implemented in different and independent
// files in folder "test_units", and all these files are included only from here, so they are compilated and tested.

#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_minimum.ipp"
#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_domain.ipp"
#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_listening_addresses.ipp"
#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_connection_addresses.ipp"
#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_tls.ipp"
#include "test_units/YamlGetDiscoveryServerParticipantConfigurationTest_get_participant_negative.ipp"

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
