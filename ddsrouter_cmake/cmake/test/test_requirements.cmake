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

# TODO
macro(test_requirements)

    include_directories("${ddsrouter_cmake_DATA_DIR}/ddsrouter_cmake/test_installer")

    check_gtest()
    check_gmock()

    if(WIN32)

    # populates out_var with the value that the PATH environment variable should have
    # in order to cover all target dependencies required for a ctest.
    # Arguments:
    # GOAL_TARGET -> dependencies related target
    # OUTPUT_VARIABLE -> name of the output variable (in parent scope) that receives the new path
    # a third argument can be specify to simplify recursive operation. If it's value is
    # "EXTRA_PATHS" the current PATH will not be included in the output.
    function(get_win32_path_dependencies GOAL_TARGET OUTPUT_VARIABLE)
        unset(LOCAL_PATH)

        get_target_property(LINK_LIBRARIES_ ${GOAL_TARGET} LINK_LIBRARIES)
        if(NOT "${LINK_LIBRARIES_}" STREQUAL "LINK_LIBRARIES_-NOTFOUND")
            foreach(LIBRARY_LINKED ${LINK_LIBRARIES_})
                if(TARGET ${LIBRARY_LINKED})
                    # Check if is a real target or a target interface
                    get_target_property(type ${LIBRARY_LINKED} TYPE)
                    if(NOT type STREQUAL "INTERFACE_LIBRARY")

                        if(LOCAL_PATH)
                            set(LOCAL_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>;${LOCAL_PATH}")
                        else(LOCAL_PATH)
                            set(LOCAL_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>")
                        endif(LOCAL_PATH)

                        # Check for dependencies recursively
                        get_win32_path_dependencies(${LIBRARY_LINKED} DEP_WIN_PATH "EXTRA_PATHS")
                        if(DEP_WIN_PATH)
                            set(LOCAL_PATH "${DEP_WIN_PATH};${LOCAL_PATH}")
                        endif(DEP_WIN_PATH)
                        unset(DEP_WIN_PATH)
                    endif()
                    unset(type)
                endif()
            endforeach()
        endif()

        if( (ARGC EQUAL 2) OR NOT (ARGV2 STREQUAL "EXTRA_PATHS"))
            set(LOCAL_PATH "${LOCAL_PATH};$ENV{PATH}")
            list(REMOVE_DUPLICATES LOCAL_PATH)
            string(REPLACE ";" "\\;" LOCAL_PATH "${LOCAL_PATH}")
        endif()

        set(${OUTPUT_VARIABLE} "${LOCAL_PATH}" PARENT_SCOPE)

    endfunction(get_win32_path_dependencies)

    else(WIN32)
        # dummy
        function(get_win32_path_dependencies)
        endfunction(get_win32_path_dependencies)
    endif(WIN32)

endmacro()
