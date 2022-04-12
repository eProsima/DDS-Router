# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

macro(add_xfail_label LIST_FILE)
    if(EXISTS ${LIST_FILE})
        file(STRINGS ${LIST_FILE} TEST_LIST)
        foreach(XFAIL_TEST ${TEST_LIST})
            set_property(TEST ${XFAIL_TEST} PROPERTY LABELS xfail)
        endforeach()
    endif()
endmacro()

macro(add_windows_xfail_label LIST_FILE)
    if(WIN32)
        add_xfail_label("${LIST_FILE}")
    endif()
endmacro()

