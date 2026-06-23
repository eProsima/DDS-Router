// Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <filesystem>
#include <fstream>
#include <iostream>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_yaml/YamlManager.hpp>
#include <ddspipe_yaml/YamlValidator.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe::yaml;

namespace test {
// Paths and files for the tests
std::string schema_path = "./ddsrouter_config_schema.json";

// Vectors with the valid and invalid YAML files
std::vector<std::string> valid_files = []()
        {
            std::vector<std::string> files;
            for (const auto& entry : std::filesystem::directory_iterator("./valid_config_files_router/"))
            {
                if (entry.path().extension() == ".yaml")
                {
                    files.push_back(entry.path().generic_string());
                }
            }
            return files;
        }();

std::vector<std::string> invalid_files = []()
        {
            std::vector<std::string> files;
            for (const auto& entry : std::filesystem::directory_iterator("./invalid_config_files_router/"))
            {
                if (entry.path().extension() == ".yaml")
                {
                    files.push_back(entry.path().generic_string());
                }
            }
            return files;
        }();
} // namespace test

/**
 * Test that a set of valid YAML configurations pass the validation
 */
TEST(YamlValidatorDdsRouterTest, validation_passed)
{
    YamlValidator validator = YamlValidator(YamlValidator::from_file(test::schema_path));

    // valid files
    {
        for (std::string st : test::valid_files)
        {
            Yaml yml = YamlManager::load_file(st);
            ASSERT_TRUE(validator.validate_YAML(yml)) << "Failed for file: " << st;
        }
    }

}

/**
 * Test that a set of invalid YAML configurations don't pass the validation
 */
TEST(YamlValidatorDdsRouterTest, validation_failed)
{
    YamlValidator validator = YamlValidator(YamlValidator::from_file(test::schema_path));

    // invalid files
    {
        for (std::string st : test::invalid_files)
        {
            Yaml yml = YamlManager::load_file(st);
            // Validate is called with false to prevent filling the output with the specific errors
            ASSERT_FALSE(validator.validate_YAML(yml, false)) << "Failed for file: " << st;
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
