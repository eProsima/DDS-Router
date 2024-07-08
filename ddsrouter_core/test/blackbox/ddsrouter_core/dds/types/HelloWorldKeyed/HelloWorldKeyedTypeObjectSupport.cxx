// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/*!
 * @file HelloWorldKeyedTypeObjectSupport.cxx
 * Source file containing the implementation to register the TypeObject representation of the described types in the IDL file
 *
 * This file was generated by the tool fastddsgen.
 */

#include "HelloWorldKeyedTypeObjectSupport.hpp"

#include <mutex>
#include <string>

#include <fastcdr/xcdr/external.hpp>
#include <fastcdr/xcdr/optional.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/xtypes/common.hpp>
#include <fastdds/dds/xtypes/type_representation/ITypeObjectRegistry.hpp>
#include <fastdds/dds/xtypes/type_representation/TypeObject.hpp>
#include <fastdds/dds/xtypes/type_representation/TypeObjectUtils.hpp>

#include "HelloWorldKeyed.hpp"


using namespace eprosima::fastdds::dds::xtypes;

// TypeIdentifier is returned by reference: dependent structures/unions are registered in this same method
void register_HelloWorldKeyed_type_identifier(
        TypeIdentifierPair& type_ids_HelloWorldKeyed)
{

    ReturnCode_t return_code_HelloWorldKeyed {eprosima::fastdds::dds::RETCODE_OK};
    return_code_HelloWorldKeyed =
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->type_object_registry().get_type_identifiers(
        "HelloWorldKeyed", type_ids_HelloWorldKeyed);
    if (eprosima::fastdds::dds::RETCODE_OK != return_code_HelloWorldKeyed)
    {
        StructTypeFlag struct_flags_HelloWorldKeyed = TypeObjectUtils::build_struct_type_flag(eprosima::fastdds::dds::xtypes::ExtensibilityKind::APPENDABLE,
                false, false);
        QualifiedTypeName type_name_HelloWorldKeyed = "HelloWorldKeyed";
        eprosima::fastcdr::optional<AppliedBuiltinTypeAnnotations> type_ann_builtin_HelloWorldKeyed;
        eprosima::fastcdr::optional<AppliedAnnotationSeq> ann_custom_HelloWorldKeyed;
        CompleteTypeDetail detail_HelloWorldKeyed = TypeObjectUtils::build_complete_type_detail(type_ann_builtin_HelloWorldKeyed, ann_custom_HelloWorldKeyed, type_name_HelloWorldKeyed.to_string());
        CompleteStructHeader header_HelloWorldKeyed;
        header_HelloWorldKeyed = TypeObjectUtils::build_complete_struct_header(TypeIdentifier(), detail_HelloWorldKeyed);
        CompleteStructMemberSeq member_seq_HelloWorldKeyed;
        {
            TypeIdentifierPair type_ids_id;
            ReturnCode_t return_code_id {eprosima::fastdds::dds::RETCODE_OK};
            return_code_id =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->type_object_registry().get_type_identifiers(
                "_int32_t", type_ids_id);

            if (eprosima::fastdds::dds::RETCODE_OK != return_code_id)
            {
                EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION,
                        "id Structure member TypeIdentifier unknown to TypeObjectRegistry.");
                return;
            }
            StructMemberFlag member_flags_id = TypeObjectUtils::build_struct_member_flag(eprosima::fastdds::dds::xtypes::TryConstructFailAction::DISCARD,
                    false, false, true, false);
            MemberId member_id_id = 0x00000000;
            bool common_id_ec {false};
            CommonStructMember common_id {TypeObjectUtils::build_common_struct_member(member_id_id, member_flags_id, TypeObjectUtils::retrieve_complete_type_identifier(type_ids_id, common_id_ec))};
            if (!common_id_ec)
            {
                EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION, "Structure id member TypeIdentifier inconsistent.");
                return;
            }
            MemberName name_id = "id";
            eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> member_ann_builtin_id;
            ann_custom_HelloWorldKeyed.reset();
            AppliedAnnotationSeq tmp_ann_custom_id;
            eprosima::fastcdr::optional<std::string> unit_id;
            eprosima::fastcdr::optional<AnnotationParameterValue> min_id;
            eprosima::fastcdr::optional<AnnotationParameterValue> max_id;
            eprosima::fastcdr::optional<std::string> hash_id_id;
            if (unit_id.has_value() || min_id.has_value() || max_id.has_value() || hash_id_id.has_value())
            {
                member_ann_builtin_id = TypeObjectUtils::build_applied_builtin_member_annotations(unit_id, min_id, max_id, hash_id_id);
            }
            if (!tmp_ann_custom_id.empty())
            {
                ann_custom_HelloWorldKeyed = tmp_ann_custom_id;
            }
            CompleteMemberDetail detail_id = TypeObjectUtils::build_complete_member_detail(name_id, member_ann_builtin_id, ann_custom_HelloWorldKeyed);
            CompleteStructMember member_id = TypeObjectUtils::build_complete_struct_member(common_id, detail_id);
            TypeObjectUtils::add_complete_struct_member(member_seq_HelloWorldKeyed, member_id);
        }
        {
            TypeIdentifierPair type_ids_index;
            ReturnCode_t return_code_index {eprosima::fastdds::dds::RETCODE_OK};
            return_code_index =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->type_object_registry().get_type_identifiers(
                "_uint32_t", type_ids_index);

            if (eprosima::fastdds::dds::RETCODE_OK != return_code_index)
            {
                EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION,
                        "index Structure member TypeIdentifier unknown to TypeObjectRegistry.");
                return;
            }
            StructMemberFlag member_flags_index = TypeObjectUtils::build_struct_member_flag(eprosima::fastdds::dds::xtypes::TryConstructFailAction::DISCARD,
                    false, false, false, false);
            MemberId member_id_index = 0x00000001;
            bool common_index_ec {false};
            CommonStructMember common_index {TypeObjectUtils::build_common_struct_member(member_id_index, member_flags_index, TypeObjectUtils::retrieve_complete_type_identifier(type_ids_index, common_index_ec))};
            if (!common_index_ec)
            {
                EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION, "Structure index member TypeIdentifier inconsistent.");
                return;
            }
            MemberName name_index = "index";
            eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> member_ann_builtin_index;
            ann_custom_HelloWorldKeyed.reset();
            CompleteMemberDetail detail_index = TypeObjectUtils::build_complete_member_detail(name_index, member_ann_builtin_index, ann_custom_HelloWorldKeyed);
            CompleteStructMember member_index = TypeObjectUtils::build_complete_struct_member(common_index, detail_index);
            TypeObjectUtils::add_complete_struct_member(member_seq_HelloWorldKeyed, member_index);
        }
        {
            TypeIdentifierPair type_ids_message;
            ReturnCode_t return_code_message {eprosima::fastdds::dds::RETCODE_OK};
            return_code_message =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->type_object_registry().get_type_identifiers(
                "anonymous_string_unbounded", type_ids_message);

            if (eprosima::fastdds::dds::RETCODE_OK != return_code_message)
            {
                {
                    SBound bound = 0;
                    StringSTypeDefn string_sdefn = TypeObjectUtils::build_string_s_type_defn(bound);
                    if (eprosima::fastdds::dds::RETCODE_BAD_PARAMETER ==
                            TypeObjectUtils::build_and_register_s_string_type_identifier(string_sdefn,
                            "anonymous_string_unbounded", type_ids_message))
                    {
                        EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION,
                            "anonymous_string_unbounded already registered in TypeObjectRegistry for a different type.");
                    }
                }
            }
            StructMemberFlag member_flags_message = TypeObjectUtils::build_struct_member_flag(eprosima::fastdds::dds::xtypes::TryConstructFailAction::DISCARD,
                    false, false, false, false);
            MemberId member_id_message = 0x00000002;
            bool common_message_ec {false};
            CommonStructMember common_message {TypeObjectUtils::build_common_struct_member(member_id_message, member_flags_message, TypeObjectUtils::retrieve_complete_type_identifier(type_ids_message, common_message_ec))};
            if (!common_message_ec)
            {
                EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION, "Structure message member TypeIdentifier inconsistent.");
                return;
            }
            MemberName name_message = "message";
            eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> member_ann_builtin_message;
            ann_custom_HelloWorldKeyed.reset();
            CompleteMemberDetail detail_message = TypeObjectUtils::build_complete_member_detail(name_message, member_ann_builtin_message, ann_custom_HelloWorldKeyed);
            CompleteStructMember member_message = TypeObjectUtils::build_complete_struct_member(common_message, detail_message);
            TypeObjectUtils::add_complete_struct_member(member_seq_HelloWorldKeyed, member_message);
        }
        CompleteStructType struct_type_HelloWorldKeyed = TypeObjectUtils::build_complete_struct_type(struct_flags_HelloWorldKeyed, header_HelloWorldKeyed, member_seq_HelloWorldKeyed);
        if (eprosima::fastdds::dds::RETCODE_BAD_PARAMETER ==
                TypeObjectUtils::build_and_register_struct_type_object(struct_type_HelloWorldKeyed, type_name_HelloWorldKeyed.to_string(), type_ids_HelloWorldKeyed))
        {
            EPROSIMA_LOG_ERROR(XTYPES_TYPE_REPRESENTATION,
                    "HelloWorldKeyed already registered in TypeObjectRegistry for a different type.");
        }
    }
}

