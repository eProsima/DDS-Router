# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# This file is part of eProsima Fast DDS Monitor.
#
# eProsima Fast DDS Monitor is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# eProsima Fast DDS Monitor is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with eProsima Fast DDS Monitor. If not, see <https://www.gnu.org/licenses/>.

option(GTEST_INDIVIDUAL "Activate the execution of GTest tests" OFF)

macro(check_gtest)
    if(NOT GTEST_FOUND)
        if(WIN32)
            option(EPROSIMA_GTEST "Activate special set of GTEST_ROOT" OFF)
            if(EPROSIMA_BUILD)
                set(EPROSIMA_GTEST ON)
            endif()
        endif()

        # Find package GTest
        if(WIN32 AND EPROSIMA_GTEST)
            if(NOT GTEST_ROOT)
                set(GTEST_ROOT_ $ENV{GTEST_ROOT})
                if(GTEST_ROOT_)
                    file(TO_CMAKE_PATH "${GTEST_ROOT_}/${MSVC_ARCH}" GTEST_ROOT)
                endif()
            else()
                file(TO_CMAKE_PATH "${GTEST_ROOT}/${MSVC_ARCH}" GTEST_ROOT)
            endif()
        endif()

        find_package(GTest)

        if(GTEST_FOUND)
            find_package(Threads REQUIRED)
            set(GTEST_LIBRARIES ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
            set(GTEST_BOTH_LIBRARIES ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
        endif()
    endif()
endmacro()

macro(check_gmock)
    if(NOT GMOCK_FOUND)
        if(WIN32)
            option(EPROSIMA_GMOCK "Activate special set of GMOCK_ROOT" OFF)
            if(EPROSIMA_BUILD)
                set(EPROSIMA_GMOCK ON)
            endif()
        endif()

        # Find package GMock
        if(WIN32 AND EPROSIMA_GMOCK)
            if(NOT GMOCK_ROOT)
                set(GMOCK_ROOT_ $ENV{GMOCK_ROOT})
                if(GMOCK_ROOT_)
                    file(TO_CMAKE_PATH "${GMOCK_ROOT_}/${MSVC_ARCH}" GMOCK_ROOT)
                endif()
            else()
                file(TO_CMAKE_PATH "${GMOCK_ROOT}/${MSVC_ARCH}" GMOCK_ROOT)
            endif()
        endif()
        find_package(GMock)

        if(GMOCK_FOUND)
            find_package(Threads REQUIRED)
            set(GMOCK_LIBRARIES ${GMOCK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
            set(GMOCK_BOTH_LIBRARIES ${GMOCK_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
        endif()
    endif()
endmacro()

macro(add_gtest)
    # Parse arguments
    if("${ARGV0}" STREQUAL "NAME")
        set(uniValueArgs NAME COMMAND)
        unset(test)
        unset(command)
    else()
        set(test "${ARGV0}")
        set(command "${test}")
    endif()
    set(multiValueArgs SOURCES ENVIRONMENTS DEPENDENCIES LABELS)
    cmake_parse_arguments(GTEST "" "${uniValueArgs}" "${multiValueArgs}" ${ARGN})

    if(GTEST_NAME)
        set(test ${GTEST_NAME})
        set(command ${GTEST_COMMAND})
    endif()

    if(GTEST_INDIVIDUAL)
        if(WIN32)
            set(WIN_PATH "$ENV{PATH}")
            get_target_property(LINK_LIBRARIES_ ${command} LINK_LIBRARIES)
            if(NOT "${LINK_LIBRARIES_}" STREQUAL "LINK_LIBRARIES_-NOTFOUND")
                foreach(LIBRARY_LINKED ${LINK_LIBRARIES_})
                    if(TARGET ${LIBRARY_LINKED})
                        # Check if is a real target or a target interface
                        get_target_property(type ${LIBRARY_LINKED} TYPE)
                        if(NOT type STREQUAL "INTERFACE_LIBRARY")
                            set(WIN_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>;${WIN_PATH}")
                        endif()
                        unset(type)
                    endif()
                endforeach()
            endif()
            foreach(DEP ${GTEST_DEPENDENCIES})
                set(WIN_PATH "$<TARGET_FILE_DIR:${DEP}>;${WIN_PATH}")
            endforeach()
            string(REPLACE ";" "\\;" WIN_PATH "${WIN_PATH}")

        endif()

        foreach(GTEST_SOURCE_FILE ${GTEST_SOURCES})
            file(STRINGS ${GTEST_SOURCE_FILE} GTEST_TEST_NAMES REGEX ^TEST)
            foreach(GTEST_TEST_NAME ${GTEST_TEST_NAMES})
                string(REGEX REPLACE ["\) \(,"] ";" GTEST_TEST_NAME ${GTEST_TEST_NAME})
                list(GET GTEST_TEST_NAME 1 GTEST_GROUP_NAME)
                list(GET GTEST_TEST_NAME 3 GTEST_TEST_NAME)
                add_test(NAME ${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}
                    COMMAND ${command} --gtest_filter=${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}:*/${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}/*)

                # Add environment
                set(GTEST_ENVIRONMENT "")
                if(WIN32)
                    set(GTEST_ENVIRONMENT "PATH=${WIN_PATH}")
                endif()

                foreach(property ${GTEST_ENVIRONMENTS})
                    list(APPEND GTEST_ENVIRONMENT "${property}")
                endforeach()

                if(GTEST_ENVIRONMENT)
                    set_tests_properties(${GTEST_GROUP_NAME}.${GTEST_TEST_NAME}
                        PROPERTIES ENVIRONMENT "${GTEST_ENVIRONMENT}")
                endif()
                unset(GTEST_ENVIRONMENT)

                # Add labels
                set_property(TEST ${GTEST_GROUP_NAME}.${GTEST_TEST_NAME} PROPERTY LABELS "${GTEST_LABELS}")

            endforeach()
        endforeach()
    else()
        add_test(NAME ${test} COMMAND ${command})

        # Add environment
        set(GTEST_ENVIRONMENT "")
        if(WIN32)
            set(WIN_PATH "$ENV{PATH}")
            get_target_property(LINK_LIBRARIES_ ${command} LINK_LIBRARIES)
            if(NOT "${LINK_LIBRARIES_}" STREQUAL "LINK_LIBRARIES_-NOTFOUND")
                foreach(LIBRARY_LINKED ${LINK_LIBRARIES_})
                    if(TARGET ${LIBRARY_LINKED})
                        # Check if is a real target or a target interface
                        get_target_property(type ${LIBRARY_LINKED} TYPE)
                        if(NOT type STREQUAL "INTERFACE_LIBRARY")
                            set(WIN_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>;${WIN_PATH}")
                        endif()
                        unset(type)
                    endif()
                endforeach()
            endif()
            foreach(DEP ${GTEST_DEPENDENCIES})
                set(WIN_PATH "$<TARGET_FILE_DIR:${DEP}>;${WIN_PATH}")
            endforeach()
            string(REPLACE ";" "\\;" WIN_PATH "${WIN_PATH}")

            set(GTEST_ENVIRONMENT "PATH=${WIN_PATH}")

        endif()

        foreach(property ${GTEST_ENVIRONMENTS})
            list(APPEND GTEST_ENVIRONMENT "${property}")
        endforeach()

        if(GTEST_ENVIRONMENT)
            set_tests_properties(${test}
                PROPERTIES ENVIRONMENT "${GTEST_ENVIRONMENT}")
        endif()
        unset(GTEST_ENVIRONMENT)

        # Add labels
        set_property(TEST ${test} PROPERTY LABELS "${GTEST_LABELS}")
    endif()
endmacro()

macro(add_xfail_label LIST_FILE)
    if(GTEST_INDIVIDUAL AND EXISTS ${LIST_FILE})
        file(STRINGS ${LIST_FILE} TEST_LIST)
        foreach(XFAIL_TEST ${TEST_LIST})
            set_property(TEST ${XFAIL_TEST} PROPERTY LABELS xfail)
        endforeach()
    endif()
endmacro()
