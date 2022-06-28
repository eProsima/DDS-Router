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

/**
 * @file TlsConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATION_HPP_
#define _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATION_HPP_

#include <string>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {
namespace security {

/**
 * @brief Enumeration of kinds of TLS configurations.
 */
enum class TlsKind
{
    inactive,
    client,
    server,
    both,
};


/**
 * Configuration holding TLS parameters.
 */
class TlsConfiguration
{
public:

    /**
     * @brief Constructs an undefined or inactive (valid nonetheless) TLSConfiguration
     *
     */
    DDSROUTER_CORE_DllAPI TlsConfiguration();

    /**
     * @brief Constructs client-oriented TLSConfiguration
     *
     * @param certificate_authority_file for the client
     *
     * @throw \c InitializationException if file is invalid.
     */
    DDSROUTER_CORE_DllAPI TlsConfiguration(
            std::string certificate_authority_file);

    /**
     * @brief Constructs server-oriented TLSConfiguration
     *
     * @param std::string private_key_file_password file for the server
     * @param std::string private_key_file file for the server
     * @param std::string certificate_chain_file file for the server
     * @param std::string dh_params_file file for the server
     *
     * @throw \c InitializationException if any file is invalid.
     */
    DDSROUTER_CORE_DllAPI TlsConfiguration(
            std::string private_key_file_password,
            std::string private_key_file,
            std::string certificate_chain_file,
            std::string dh_params_file);

    /**
     * @brief Constructs client-and-server-oriented TLSConfiguration
     *
     * @param certificate_authority_file for the client
     * @param std::string private_key_file_password file for the server
     * @param std::string private_key_file for the server
     * @param std::string certificate_chain_file for the server
     * @param std::string dh_params_file for the server
     *
     * @throw \c InitializationException if any file is invalid.
     */
    DDSROUTER_CORE_DllAPI TlsConfiguration(
            std::string certificate_authority_file,
            std::string private_key_file_password,
            std::string private_key_file,
            std::string certificate_chain_file,
            std::string dh_params_file);

    /**
     * @brief Returns whether configuration is active
     */
    DDSROUTER_CORE_DllAPI bool is_active() const noexcept;

    /**
     * @brief Check configuration kind compatibility
     *
     * @tparam Kind TlsConfiguration kind
     *
     * @return Whether configuration is compatible with parameter \c TlsKind
     */
    template <TlsKind Kind>
    DDSROUTER_CORE_DllAPI bool compatible() const noexcept;

    /**
     * @brief certificate_authority_file getter
     *
     * @throw \c InconsistencyException if configuration is not client-compatible
     */
    DDSROUTER_CORE_DllAPI const std::string& certificate_authority_file() const;

    /**
     * @brief private_key_file_password getter
     *
     * @throw \c InconsistencyException if configuration is not server-compatible
     */
    DDSROUTER_CORE_DllAPI const std::string& private_key_file_password() const;

    /**
     * @brief private_key_file getter
     *
     * @throw \c InconsistencyException if configuration is not server-compatible
     */
    DDSROUTER_CORE_DllAPI const std::string& private_key_file() const;

    /**
     * @brief certificate_chain_file getter
     *
     * @throw \c InconsistencyException if configuration is not server-compatible
     */
    DDSROUTER_CORE_DllAPI const std::string& certificate_chain_file() const;

    /**
     * @brief dh_params_file getter
     *
     * @throw \c InconsistencyException if configuration is not server-compatible
     */
    DDSROUTER_CORE_DllAPI const std::string& dh_params_file() const;

private:

    //! Internal throwing check
    template <TlsKind Kind>
    void check_valid_() const;

    ////////
    // MEMBERS

    //! Configuration Kind
    TlsKind kind_;

    ////////
    // CLIENT-ONLY VARIABLES
    std::string certificate_authority_file_;

    ////////
    // SERVER-ONLY VARIABLES
    std::string private_key_file_password_;
    std::string private_key_file_;
    std::string certificate_chain_file_;
    std::string dh_params_file_;
};

} /* namespace security */
} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATION_HPP_ */
