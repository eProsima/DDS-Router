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
 * @file DomainId_configuration.cpp
 *
 */

#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/security/tls/TlsConfiguration.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

ParticipantId YamlElementConfiguration::participant_id(const Yaml& yaml)
{
    if (yaml[PARTICIPANT_NAME_TAG])
    {
        std::string name_str = yaml[PARTICIPANT_NAME_TAG].as<std::string>();
        return ParticipantId(name_str);
    }
    else
    {
        throw ConfigurationException("Id not specified.");
    }
}

ParticipantKind YamlElementConfiguration::participant_type(const Yaml& yaml)
{
    if (yaml[PARTICIPANT_TYPE_TAG])
    {
        std::string type_str = yaml[PARTICIPANT_TYPE_TAG].as<std::string>();
        return ParticipantKind::participant_type_from_name(type_str);
    }
    else
    {
        throw ConfigurationException("Type not specified.");
    }
}

security::TlsConfiguration YamlElementConfiguration::tls_configuration(const Yaml& yaml)
{
    bool has_private_key_file = false;
    bool has_certificate_authority_file = false;
    bool has_certificate_chain_file = false;
    bool has_dh_params_file = false;

    std::string private_key_file_password;
    std::string private_key_file;
    std::string certificate_authority_file;
    std::string certificate_chain_file;
    std::string dh_params_file;

    try
    {
        if (yaml[TLS_CA_TAG])
        {
            has_certificate_chain_file = true;
            certificate_authority_file = yaml[TLS_CA_TAG].as<std::string>();
        }

        if (yaml[TLS_CERT_TAG])
        {
            has_certificate_authority_file = true;
            certificate_chain_file = yaml[TLS_CERT_TAG].as<std::string>();
        }

        if (yaml[TLS_DHPARAMS_TAG])
        {
            has_dh_params_file = true;
            dh_params_file = yaml[TLS_DHPARAMS_TAG].as<std::string>();
        }

        if (yaml[TLS_PRIVATE_KEY_TAG])
        {
            has_private_key_file = true;
            private_key_file = yaml[TLS_PRIVATE_KEY_TAG].as<std::string>();
        }

        if (yaml[TLS_PASSWORD_TAG])
        {
            private_key_file_password = yaml[TLS_PASSWORD_TAG].as<std::string>();
        }
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException("Error format in TLS configuration.");
    }

    if (has_private_key_file)
    {
        if (has_certificate_authority_file)
        {
            // Both TLS configuration
            return security::TlsConfiguration(
                private_key_file_password,
                private_key_file,
                certificate_authority_file,
                certificate_chain_file,
                dh_params_file);
        }
        else
        {
            // Server TLS configuration
            return security::TlsConfiguration(
                private_key_file_password,
                private_key_file,
                certificate_chain_file,
                dh_params_file);
        }
    }
    else
    {
        // Client TLS configuration
        return security::TlsConfiguration(certificate_authority_file);
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
