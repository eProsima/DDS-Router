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
// limitations under the License\.

#pragma once

#include <string>
#include <memory>

#include <fastdds/rtps/transport/TCPTransportDescriptor.h>

#include <cpp_utils/macros/custom_enumeration.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_core/configuration/IConfiguration.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace types {

/**
 * @brief Enumeration of kinds of TLS configurations.
 */
ENUMERATION_BUILDER(
    TlsKind,
    inactive,
    client,
    server,
    both
    );

/**
 * Configuration holding TLS parameters.
 */
struct TlsConfiguration : public core::IConfiguration
{

    /**
     * @brief Constructs an undefined or inactive (valid nonetheless) TLSConfiguration
     */
    DDSPIPE_PARTICIPANTS_DllAPI TlsConfiguration() = default;

    /**
     * @brief Returns whether configuration is active
     */
    DDSPIPE_PARTICIPANTS_DllAPI bool is_active() const noexcept;

    /**
     * @brief Check configuration kind compatibility
     *
     * @tparam Kind TlsConfiguration kind
     *
     * @return Whether configuration is compatible with parameter \c TlsKind
     *
     * @note this function is implemented in hpp to avoid a new file for only this method.
     */
    template <TlsKind Kind>
    bool compatible() const noexcept
    {
        return this->kind != TlsKind::inactive &&
               (Kind == this->kind || (this->kind == TlsKind::both && Kind != TlsKind::inactive));
    }

    template <TlsKind Kind>
    DDSPIPE_PARTICIPANTS_DllAPI bool is_valid_kind(
            utils::Formatter& error_msg) const noexcept;

    //! Activate TLS configuration in a TCP transport descriptor
    DDSPIPE_PARTICIPANTS_DllAPI void enable_tls(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
            bool client = false) const;

    //! Activate TLS client configuration in a TCP transport descriptor
    DDSPIPE_PARTICIPANTS_DllAPI void enable_tls_client(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
            bool only_client) const;

    //! Activate TLS server configuration in a TCP transport descriptor
    DDSPIPE_PARTICIPANTS_DllAPI void enable_tls_server(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor) const;

    DDSPIPE_PARTICIPANTS_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    ////////
    // MEMBERS

    //! Configuration Kind
    TlsKind kind = TlsKind::inactive;
    bool verify_peer = true;

    ////////
    // CLIENT-ONLY VARIABLES
    std::string certificate_authority_file = "";
    std::string sni_server_name = "";

    ////////
    // SERVER-ONLY VARIABLES
    std::string private_key_file_password = "";
    std::string private_key_file = "";
    std::string certificate_chain_file = "";
    std::string dh_params_file = "";
};

} /* namespace types */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
