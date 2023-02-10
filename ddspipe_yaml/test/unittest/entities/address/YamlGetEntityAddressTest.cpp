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

#include "test_units/YamlGetEntityAddressTest_get_transport_protocol.ipp"
#include "test_units/YamlGetEntityAddressTest_get_ip_version.ipp"
#include "test_units/YamlGetEntityAddressTest_get_port.ipp"
#include "test_units/YamlGetEntityAddressTest_get_ip.ipp"
#include "test_units/YamlGetEntityAddressTest_get_address_ip.ipp"
#include "test_units/YamlGetEntityAddressTest_get_address_domain.ipp"
#include "test_units/YamlGetEntityAddressTest_get_address_defaults.ipp"
#include "test_units/YamlGetEntityAddressTest_ip_and_domain.ipp"

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
