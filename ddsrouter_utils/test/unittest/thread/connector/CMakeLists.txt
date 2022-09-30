
###################################
# One Shot Connector Test
###################################

set(TEST_NAME
    OneShotConnectorTest)

set(TEST_SOURCES
        one_shot_connector_test.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/task/OwnedTask.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/StdThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
    )

set(TEST_LIST
        one_shot_test_no_params
        one_shot_test_int
        one_shot_test_string
        one_shot_test_bool_int_string
        one_shot_test_complex_args
    )

set(TEST_EXTRA_LIBRARIES
        ${MODULE_DEPENDENCIES}
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

###################################
# Slot Connector Test
###################################

set(TEST_NAME
    SlotConnectorTest)

set(TEST_SOURCES
        slot_connector_test.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/task/OwnedTask.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/StdThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
    )

set(TEST_LIST
        slot_test_no_params
        slot_test_int
        slot_test_string
        slot_test_bool_int_string
        slot_test_complex_args
    )

set(TEST_EXTRA_LIBRARIES
        ${MODULE_DEPENDENCIES}
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
