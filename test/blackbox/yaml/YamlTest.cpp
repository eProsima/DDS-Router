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

#include <algorithm>
#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <TestLogHandler.hpp>

#include <ddsrouter/yaml/YamlConfigurationTest_.hpp>
#include <ddsrouter/yaml/YamlConfiguration_.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

TEST(YamlTest, test)
{
    // Yaml yml;
    // yml["a"] = "b";
    // YamlConfigurationNative finder(yml, "a");

    // ASSERT_TRUE(finder.found());
    // ASSERT_EQ(finder.find<std::string>(), "b");

    // YamlConfigurationNative finder2(yml, "b");
    // ASSERT_FALSE(finder2.found());
    // // ASSERT_EQ(finder2.find(), "b");

    // Yaml yml2;
    // yml2["d"] = 3;

    // YamlConfiguration domain_finder(yml2, "d");
    // DomainId id = domain_finder.find<DomainId>();
    // ASSERT_EQ(id, DomainId(static_cast<DomainIdType>(3)));
    // ASSERT_NE(id.domain_id(), DomainId(static_cast<DomainIdType>(2)).domain_id());
}

TEST(YamlTest, test2)
{
    // Yaml yml_x;
    // Yaml yml;
    // yml_x["x"] = 1;
    // yml_x["y"] = "y";
    // yml["position"] = yml_x;

    // std::pair<int, std::string> p = testy::YamlConfiguration_2::get<std::pair<int, std::string>>(yml, "position");

    // ASSERT_EQ(p.first, 1);
    // ASSERT_EQ(p.second, "y");
}

TEST(YamlTest, test3)
{
    // Yaml yml_d;
    // Yaml yml;
    // yml_d["domain"] = 2;
    // yml["x"] = yml_d;

    // DomainId d = testy::YamlConfiguration_2::get<DomainId>(yml, "x");

    // ASSERT_EQ(d, DomainId(2u));
    // ASSERT_NE(d.domain_id(), DomainId(3u).domain_id());
}

TEST(YamlTest, test4)
{
    Yaml yml_d;
    Yaml yml;
    yml_d["domain"] = 2;
    yml["x"] = yml_d;

    DomainId d = YamlConfiguration::get<DomainId>(yml, "x");
    DomainId d2 = YamlConfiguration::get<DomainId>(yml_d);

    ASSERT_EQ(d, DomainId(2u));
    ASSERT_EQ(d, d2);
    ASSERT_NE(d.domain_id(), DomainId(3u).domain_id());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
