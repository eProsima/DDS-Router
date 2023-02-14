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

#pragma once

#include <cpp_utils/types/Fuzzy.hpp>
#include <cpp_utils/enum/EnumBuilder.hpp>

#include <ddspipe_yaml/library/library_dll.h>
#include <ddspipe_yaml/Yaml.hpp>

namespace eprosima {
namespace ddspipe {
namespace yaml {

/**
 * @brief Yaml Configuration Version
 *
 * With each version, the yaml format could change in different aspects
 * - new fields
 * - eliminate fields
 * - change key words / tags
 * - change requirement of field
 * - etc.
 *
 * Each version is stored in a value of this enumeration, and each \c get method (without tag) is speciallized
 * for each different versions.
 *
 * There is a version call \c LATEST that refers to the lates valid Yaml Configuration Versions.
 * Most elemnts will not change from a version change to other. Thus, all of them must be implemented as
 * the \c LATEST version, and then if a specific object changes with a specific version, implement that version
 * (not latest) separately.
 * This way, any new version uses \c LATEST as default implementation, and old versions that diverged from latest
 * have their own implementation. As old versions will not change, those methods are definitive, and do not
 * need further maintenance.
 */
enum YamlReaderVersion
{
    /**
     * @brief First version.
     *
     * @version 0.1.0
     * @version 0.2.0
     */
    V_1_0,

    /**
     * @brief  Version 2.0
     *
     * @version 0.3.0
     *
     * - Adds builtin-topics tag.
     * - Adds participants list.
     * - Changes the parent of guid for DS to a new tag discovery-server-guid.
     * - Adds domain tag in Address to remplace ip when DNS.
     */
    V_2_0,

    /**
     * @brief  Latest version.
     *
     * @version 0.4.0
     *
     * - Change wan to initial peers participant
     * - Add Specs
     */
    V_3_0,

    /**
     * @brief  Main version.
     *
     * This is the version used when the method is not specialized regarding the version,
     * or the latest version when it does.
     */
    LATEST,
};

using TagType = std::string;

/**
 * @brief Class that encapsulates methods to get values from Yaml Node.
 *
 * Every method is implemented
 */
class DDSPIPE_YAML_DllAPI YamlReader
{
public:

    /**
     * @brief Whether key \c tag is present in \c yml
     *
     * The key \c tag must be in base level of \c yml , it will no be looked for recursively.
     *
     * It could only look for keys in yaml map and empty yaml. It will fail in array and scalar cases.
     * In this second case, return will always be false.
     *
     * @param yml base yaml
     * @param tag key to look for in \c yml
     * @return true id \c tag is a key in \c yml
     * @return false otherwise
     *
     * @throw \c ConfigurationException if \c yml is not a map or empty yaml
     */
    static bool is_tag_present(
            const Yaml& yml,
            const TagType& tag);

    /**
     * @brief Get the yaml inside key \c tag in \c yml
     *
     * \c tag must be present in \c yml . It uses \c is_tag_present to check the key is present.
     * Use \c is_tag_present before calling this method in order to avoid the exception.
     *
     * @param yml base yaml
     * @param tag key refering the value returned
     * @return yaml value inside key \c tag
     *
     * @throw \c ConfigurationException if \c tag is not present in \c yml
     */
    static Yaml get_value_in_tag(
            const Yaml& yml,
            const TagType& tag);

    //! TODO comment
    template <typename T>
    static T get(
            const Yaml& yml,
            const YamlReaderVersion version);

    //! Get element inside \c tag
    template <typename T>
    static T get(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version);

    /**
     * @brief Fill an element given by parameter with the values inside \c yml
     *
     * This method simplifies the process of retrieving an object whose parent has its own \c fill method,
     * as the code is reused from one another calling parent \c fill in child.
     * It is also very helpful to handle default creation values. Without this, every different default value
     * must have its own if-else clause, forcing to create the respective constructor. With this method,
     * the default values are initialized with the default constructor, and then are overwritten by the yaml.
     * [this problem arises because C++ does not allow different order of parameters in method call]
     */
    template <typename T>
    static void fill(
            T& object,
            const Yaml& yml,
            const YamlReaderVersion version);

    //! Fill an element given by parameter with the values inside \c tag in \c yml
    template <typename T>
    static void fill(
            T& object,
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version);

    //! TODO comment
    template <typename T>
    static std::list<T> get_list(
            const Yaml& yml,
            const YamlReaderVersion version);

    //! Get list inside \c tag
    template <typename T>
    static std::list<T> get_list(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version);

    /**
     * @brief Get set inside \c tag
     *
     * It get a list of elements T with \c get_list and then convert it to set.
     *
     * @tparam T type of each object of the set
     * @param yml base yaml
     * @param tag key to yaml containing set
     * @param version configuration version
     * @return set of elements
     */
    template <typename T>
    static std::set<T> get_set(
            const Yaml& yml,
            const TagType& tag,
            const YamlReaderVersion version);

    //! TODO comment
    template <typename T>
    static T get_scalar(
            const Yaml& yml);

    //! Get scalar value inside \c tag
    template <typename T>
    static T get_scalar(
            const Yaml& yml,
            const TagType& tag);

    //! TODO comment
    template <typename T>
    static T get_enumeration(
            const Yaml& yml,
            const std::map<TagType, T>& enum_values);

    //! Get enumeration value inside \c tag
    template <typename T>
    static T get_enumeration(
            const Yaml& yml,
            const TagType& tag,
            const std::map<TagType, T>& enum_values);

    //! TODO comment
    template <typename T>
    static T get_enumeration_from_builder(
            const Yaml& yml,
            const utils::EnumBuilder<T>& enum_builder);

    //! Get enumeration value inside \c tag
    template <typename T>
    static T get_enumeration_from_builder(
            const Yaml& yml,
            const TagType& tag,
            const utils::EnumBuilder<T>& enum_builder);
};

/**
 * @brief \c YamlReaderVersion to stream serialization
 */
std::ostream& operator <<(
        std::ostream& os,
        const YamlReaderVersion& version);

} /* namespace yaml */
} /* namespace ddspipe */
} /* namespace eprosima */

// Include implementation template file
#include <ddspipe_yaml/impl/YamlReader.ipp>
