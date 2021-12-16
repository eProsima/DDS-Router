# Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
"""
Script to execute and validate Discovery Server v2 tests.

This script encapsulate several funciontalities focus on testing the
Discovery Server v2.

These tests are parametrized from tests_params, a file where all
tests cases are described in json format, so this script knows how to
execute and validate each of them.

The main functionality is to execute each of the tests in parameter file
once per combination of configurations possible.

For each configuration, each test will be divided in different simultaneous
process that will execute in different threads.

For each process a tool is executed (DS tool or fastdds tool) following the
configurations given: configuration file, environment variable, tool args.

For each process a validation is run in order to check that the process
has been executed successfully and has generated a correct output.
These validators are also parametrized in test_params, because each process
expects a different behaviour.
"""
import argparse
import asyncio
import functools
import glob
import itertools
import json
import logging
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

from asyncio.subprocess import PIPE

import shared.shared as shared

import validation.validation as val

DESCRIPTION = """Script to execute and validate Discovery Server v2 test"""
USAGE = ('python3 run_test.py -e <path/to/discovery-server/executable>'
         ' [-p <path/to/test/parameteres/file>] [-t LIST[test-name]]'
         ' [-f <path/to/fastd)ds/tool>] [-d] [-r] [-i <bool>] [-s <bool>]'
         ' [--force-remove]')

# Max default time to kill a process in case it gets stucked
# This is done by ctest automatically, but this script could be
# run independently of ctest
MAX_TIME = 60*5


def parse_options():
    """
    Parse arguments.

    :return: The arguments parsed.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        add_help=True,
        description=(DESCRIPTION),
        usage=(USAGE)
    )
    required_args = parser.add_argument_group('required arguments')
    required_args.add_argument(
        '-e',
        '--exe',
        type=str,
        required=True,
        help='Path to ddsrouter executable.'
    )
    parser.add_argument(
        '-p',
        '--params',
        type=str,
        required=False,
        default=os.path.join(
            pathlib.Path(__file__).parent.absolute(),
            os.path.join('configuration', 'tests_params.json')),
        help='Path to the json file which contains the tests parameters.'
    )
    parser.add_argument(
        '-t',
        '--test',
        nargs='+',
        help='Name of the test case to execute (without xml extension) '
             'or a pattern of the test name'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )
    parser.add_argument(
        '-r',
        '--not-remove',
        action='store_true',
        help='Do not remove generated files at the end of the execution. '
             'In case of test failure, files are not removed'
    )
    parser.add_argument(
        '--force-remove',
        action='store_true',
        help='Remove generated files in any result case'
    )

    return parser.parse_args()


def working_directory():
    """Return the working directory path."""
    return os.getcwd()


async def read_output(output, num_lines, index):
    """
    Read an process stream output, printing each line using the internal log.
    Also update the line counter in the num_lines list using the index argument.

    :param[in] output: Process stream output.
    :param[inout] num_lines List with line counters for each process stream output.
    :param[in] index Indicates which line counter must be updated.
    """

    while True:
        try:
            line = await asyncio.wait_for(output.readline(), timeout=None)
        except asyncio.CancelledError:
            pass
        else:
            if line:
                num_lines[index] = num_lines[index] + 1
                logger.info(line.decode('utf-8'))
                continue
        break


async def read_outputs(proc, num_lines):
    """
    Read asynchronously the stdout and stderr of the process.

    :param[in] proc Process whose stream outputs will be read.
    :param[inout] num_lines List with line counters for each process stream output.
    """
    await asyncio.gather(read_output(proc.stdout, num_lines, 0), read_output(proc.stderr, num_lines, 1))


async def run_command(process_args, environment, timeout):
    """
    Execute a process and read its stream outputs asynchronouly.

    :param[in] process_args List of process arguments.
    :param[in] environment List of environment variables to be used when executing the process.
    :param[in] timeout Expiration time of the execution.

    :return Tuple (process return code, lines printed on stderr stream output)
    """
    proc = await asyncio.create_subprocess_exec(
        *process_args,
        env=environment,
        stdout=PIPE,
        stderr=PIPE
    )

    num_lines = [0, 0]

    try:
        await asyncio.wait_for(read_outputs(proc, num_lines), timeout)
    except asyncio.TimeoutError:
        pass

    try:
        proc.send_signal(signal.SIGINT)
        time.sleep(1)
    except Exception:
        pass

    try:
        proc.kill()
    except Exception:
        pass

    return (await proc.wait(), num_lines[1])


def pass_shm_contrains():
    """
    Check /dev/shm is greater than 64Mb (docker default).
    """
    ret_value = True
    if sys.platform == 'linux':
        proc = subprocess.check_output("df --output=size /dev/shm | tail -n 1", shell=True).decode()
        size = int(proc)
        if 65536 >= size:
            print('Test cannot run in this system. Contrain shm_size not pass.')
            ret_value = False

    return ret_value


def execute_validate_thread_test(
    process_id,
    test_id,
    result_list,
    ds_tool_path,
    process_params,
    config_file,
    flags_in,
    fds_path=None,
    clear=True,
    debug=False
):
    """
    Execute a single process inside a test and validate it.

    This function will get all the needed information from a process
    paramenter json object, and will creat the environment variables, the
    arguments for the tool and the initial and final time for a single process.
    It will execute the process and validate it afterwards.

    :param process_id: process unique identifier
    :param test_id: test identifier
    :param result_list: boolean list to append result
        (threading does not support function return).
    :param ds_tool_path: path to Discovery-Server tool.
    :param process_params: process parameters.
    :param config_file: xml file for default configuration of FastDDS.
    :param flags_in: auxiliar flags to use with the DS tool.
    :param fds_path: path to fastdds tool. This arg is not needed unless
        test must execute fastdds tool, in which case it will raise an error
        if it is not set (Default: None).
    :param clear: if true remove generated files if test passes.
    :param debug: Debug flag (Default: False).

    :return: True if everything OK, False if any error or if test did not pass
    """
    ##########################
    # PARAMENTER CONFIGURATION

    process_name = test_id + '_' + process_id
    logger.debug(f'Getting paramenters for process test {process_name}')

    # Get creation time (default 0)
    creation_time = process_params.get('creation_time', 0)

    # Get kill time (default MaxTime)
    kill_time = process_params.get('kill_time', MAX_TIME)

    # Get  environment variables
    try:
        env_var = [(env['name'], env['value']) for env in
                   process_params['environment_variables']]
    except KeyError:
        env_var = []

    # Check whether it must execute the DS tool or the fastdds tool
    xml_config_file = None
    result_file = None
    if 'xml_config_file' in process_params.keys():
        # DS tool with xml config file
        xml_config_file = process_params['xml_config_file']
        result_file = val.validation_file_name(process_name)

    elif 'tool_config' in process_params.keys():
        # fastdds tool
        server_id = 0
        server_address = None
        server_port = None
        if fds_path is None:
            logger.error('Non tool given and needed')
            result_list.append(False)
            return False

        try:
            # Try to set args for fastdds tool
            # If any is missing could not be an error
            server_id = process_params['tool_config']['id']
            server_address = process_params['tool_config']['address']
            server_port = process_params['tool_config']['port']
        except KeyError:
            pass

        result_file = 'servertool_' + str(server_id)

    else:
        logger.error(f'Incorrect process paramenters: {process_name}')
        result_list.append(False)
        return False

    # Flags to add to command
    flags = [] + flags_in  # avoid list copy
    if 'flags' in process_params.keys():
        logger.debug(f'Adding flags to execution: {process_params["flags"]}')
        flags.extend(process_params['flags'])

    ###########
    # EXECUTION
    # Wait for initial time
    time.sleep(creation_time)

    # Set env var
    my_env = os.environ.copy()
    for name, value in env_var:
        logger.debug(f'Adding envvar {name}:{value} to {process_name}')
        my_env[name] = value
    # Set configuration file
    my_env['FASTRTPS_DEFAULT_PROFILES_FILE'] = config_file
    logger.debug(f'Configuration file {config_file} to {process_name}')

    # Launch
    if xml_config_file is not None:
        # Create args with config file and outputfile
        process_args = \
            [ds_tool_path, '-c', xml_config_file, '-o', result_file] + flags

    else:
        # Fastdds tool
        process_args = [fds_path, '-i', str(server_id)]
        if server_address is not None:
            process_args.append('-l')
            process_args.append(str(server_address))
        if server_port is not None:
            process_args.append('-p')
            process_args.append(str(server_port))

    # Execute
    logger.debug(f'Executing process {process_id} in test {test_id} with '
                 f'command {process_args}')
    process_ret, lines = asyncio.run(run_command(process_args, my_env, kill_time))

    # Do not use communicate, as stderr is needed further in validation

    ############
    # VALIDATION

    # Check if validation needed by test params
    if 'validation' not in process_params.keys():
        result_list.append(True)
        return True

    # Validation needed, pass to validate_test function
    logger.debug(f'Executing validation for process {process_name}')
    validation_params = process_params['validation']
    validator_input = val.ValidatorInput(
        process_ret,
        lines,
        result_file
    )

    # Call validate_test to validate with every validator in parameters
    result = val.validate_test(
        process_name,
        validation_params,
        validator_input,
        debug,
        logger
    )

    #######
    # CLEAR
    # In case we must clear the generated file
    if result and clear:
        clear_file(working_directory(), result_file)

    # Update result_list and return
    result_list.append(result)
    return result


def execute_validate_test(
    test_name,
    test_id,
    ds_tool_path,
    test_params,
    config_file,
    flags,
    fds_path=None,
    clear=True,
    debug=False
):
    """
    Execute every process test and validate within parameters given.

    It executes every process in a different independent thread.
    It launch every process and wait till all of them are finished.
    Afterwards, it checks whether all processes has finished successfully

    :param test_name: test name.
    :param test_id: unique test identifier with test name and configurations.
    :param ds_tool_path: path to Discovery-Server tool.
    :param test_params: test parameters.
    :param config_file: xml file for default configuration of FastDDS.
    :param flags: auxiliar flags to use with the DS tool.
    :param fds_path: path to fastdds tool. This arg is not needed unless
        test must execute fastdds tool, in which case it will raise an error
        if it is not set (Default: None).
    :param clear: if true remove generated files if test passes.
    :param debug: Debug flag (Default: False).

    :return: True if everything OK, False if any error or if test did not pass
    """
    try:
        processes = test_params['processes']
    except KeyError:
        logger.error('Missing processes in test parameters')
        return False

    # List of thread configurations to run every test process
    thread_list = []
    # List of results for every test process. Each process will append its
    # result and the it will join the results (Python lists are thread safe)
    result_list = [True]

    logger.debug(f'Preparing processes for test: {test_id}')

    # Read every process config and run it in different threads
    for process_id, process_config in processes.items():

        thread = threading.Thread(
            target=execute_validate_thread_test,
            args=(process_id,
                  test_id,
                  result_list,
                  ds_tool_path,
                  process_config,
                  config_file,
                  flags,
                  fds_path,
                  clear,
                  debug))
        thread_list.append(thread)

    logger.debug(f'Executing process for test: {test_id}')

    # Start threads
    for thread in thread_list:
        thread.start()

    # Wait for all processes to end
    for thread in thread_list:
        thread.join()

    logger.debug(f'Finished all processes for test: {test_id}')

    result = functools.reduce(lambda a, b: a and b, result_list)

    val.print_result_bool(
        logger,
        f'Result for test {test_id}: ',
        result
    )

    # Clear /dev/shm
    subprocess.call('rm -f /dev/shm/*fastrtps*', shell=True)

    return result


def get_configurations(config_params, intraprocess, shm):
    """
    Extract configurations from json.

    It gets the information related to the test configurations from the
    configuration tag in json parameter file.
    It set the configuration files possible.
    It also creates the combinatory for flags to use in each call. This means,
    it gets all possible flags callable and mix them to test any possible
    case.

    :param config_params: dictionary with configurations.
    :param intraprocess: only use intraprocess as configuration file.
    :param shm: only use shared memory as default transport.

    :return: tuple of two arrays.
        First array is an array of tuples where first
        element is the name of the configuration and the second element is
        the path to the configuration
        Second element is an array of tuples where first element is the
        name of the flags combination, and second is an array of the
        flags that will be used in this combination
    """
    # Get configuration files
    config_files = []
    try:
        configs = config_params['configuration_files']
        for k, v in configs.items():
            config_files.append((k, v))
    except KeyError as e:
        logger.debug(e)

    # If intraprocess arg is set only use that specific configuration file
    if intraprocess is not None:
        if intraprocess:
            config_files = \
                [c for c in config_files if c[0] == 'INTRAPROCESS_ON']
        else:
            config_files = \
                [c for c in config_files if c[0] == 'INTRAPROCESS_OFF']

    # If no configuration files given, create CONF by default
    if len(config_files) == 0:
        config_files.append(('NO_CONFIG', ''))

    logger.debug(f'Different configuration files to execute: '
                 f'{[c[0] for c in config_files]}')

    # Create flag parameters for DS tool execution
    flags = []
    try:
        fl = config_params['flags']
        for k, v in fl.items():
            flags.append((k, v))
    except KeyError as e:
        logger.debug(e)

    # In case arg shm is set, there is not combinatory but an strict flag
    if shm is not None and not shm:
        flags = [f for f in flags if f[0] == 'SHM_OFF']
    if shm is not None and shm:
        flags = [f for f in flags if f[0] != 'SHM_OFF']

    flags_combinatory = []
    for i in range(1, 1+len(flags)):
        for combination in itertools.combinations(flags, i):
            comb_name = '.'.join([c[0] for c in combination])
            comb_flags = [c[1] for c in combination]
            flags_combinatory.append((comb_name, comb_flags))

    # Default flag in case there has not been any flag combination and
    # shm arg is not given
    if len(flags_combinatory) == 0 or shm is None:
        flags_combinatory.append(('NO_FLAGS', []))

    logger.debug(f'Different flags to execute: '
                 f'{[f[0] for f in flags_combinatory]}')

    return config_files, flags_combinatory


def create_tests(
    params_file,
    config_params,
    discovery_server_tool_path,
    tests=None,
    intraprocess=None,
    shm=None,
    clear=True,
    fds_path=None,
    debug=False,
):
    """
    Create test combinatory and execute each test.

    It gets the different tests that are going to be executed, and the
    different configurations and flags that are going to take, and it will
    iterate over all of them once to execute and validate each one of them.

    :param params_file_df: tests paramenters.
    :param config_params: configuration paramenters.
    :param discovery_server_tool_path: discovery server tool path.
    :param tests: array of strings with test names or patterns of test names.
    :param intraprocess: if set it specifies if intraprocess is used or not.
        If None, both with and without intraprocess will be executed.
        (Default: None)
    :param shm: if set it specifies if shared memory is used or not.
        If None, both with and without shared memory will be executed.
        (Default: None)
    :param clear: if true remove generated files if test passes.
    :param fds_path: path to fastdds tool. This arg is not needed unless
        test must execute fastdds tool, in which case it will raise an error
        if it is not set (Default: None).
    :param debug: Debug flag (Default: False).

    :return: True if all the tests passed the validation, False otherwise.
    """
    # get all tests available in parameter file
    if tests is None:
        tests = set(params_file.keys())
    else:
        ex_tests = set()
        for t in tests:
            # Add any test matching this string
            ex_tests.update(
                [test for test in params_file.keys() if test.find(t) != -1]
            )
        tests = ex_tests

    # Get Test file and name. Duplicated to follow other parameter
    # configuration patterns
    tests = [(t, t) for t in sorted(tests)]

    logger.debug(f'Different tests to execute: {[t[0] for t in tests]}')

    # Get configurations
    config_files, flags_combinatory = get_configurations(
        config_params,
        intraprocess,
        shm
    )

    test_results = True

    # iterate over parameters
    for test_name, test in tests:
        for config_name, config_file in config_files:
            for flag_name, flags in flags_combinatory:
                # Check constrains
                # - shm_size
                if shm is not None and shm:
                    try:
                        if (params_file[test]['contrains'] and 'shm_size' in params_file[test]['contrains'] and
                                not pass_shm_contrains()):
                            continue
                    except KeyError:
                        pass

                # Remove Databases if param <clear> set in test params in order
                # to execute same process again and not load old databases
                # Last database will stay except clear flag is set
                try:
                    if params_file[test]['clear']:
                        clear_db(working_directory())
                except KeyError:
                    pass

                test_id = '' + test_name
                test_id += '.' + config_name
                test_id += '.' + flag_name

                logger.info('--------------------------------------------')
                logger.info(f'Running {test}'
                            f' with config file <{config_file}>'
                            f' and flags {flags}')

                test_results &= execute_validate_test(
                        test_name,
                        test_id,
                        discovery_server_tool_path,
                        params_file[test],
                        config_file,
                        flags,
                        fds_path,
                        clear,
                        debug) and test_results
                logger.info('--------------------------------------------')

    return test_results


def clear_db(wd):
    """
    Clear the working directory removing the generated database files.

    Some processes generate databases in working directory for persistence
    prupose. These files could break another process run, so they should be
    deleted.
    These files are called *.json or *.db, so any other file in the wd
    with such name will be erased.

    :param wd: The working directory where clear the generated files.
    """
    files = glob.glob(
        os.path.join(wd, '*.json'))
    files.extend(glob.glob(
        os.path.join(wd, '*.db')))

    logger.debug(f'Removing files: {files}')

    for f in files:
        try:
            os.remove(f)
        except OSError:
            logger.error(f'Error while deleting file: {f}')


def clear(wd):
    """
    Clear the working directory removing the generated files.

    Remove each file that matches pattern *.snapshot~, *.json or *.db

    :param wd: The working directory where clear the generated files.
    """
    files = glob.glob(
        os.path.join(wd, '*.snapshot~'))
    files.extend(glob.glob(
        os.path.join(wd, '*.json')))
    files.extend(glob.glob(
        os.path.join(wd, '*.db')))

    logger.debug(f'Removing files: {files}')

    for f in files:
        try:
            os.remove(f)
        except OSError:
            logger.error(f'Error while deleting file: {f}')


def clear_file(wd, file_name):
    """
    Clear an specific file in the directory.

    :param wd: The working directory where clear the file.
    :param file_name: Name of file to remove.
    """
    files = glob.glob(
        os.path.join(wd, file_name))

    logger.debug(f'Removing files: {files}')

    for f in files:
        try:
            os.remove(f)
        except OSError:
            logger.error(f'Error while deleting file: {f}')


def load_test_params(tests_params_path):
    """
    Load test parameters.

    :param tests_params_path: The path to the json file which contains the
        test parameters.

    :return: Tuple of Python dicts, first test parameters and second
        configuration paramenters.
    """
    logger.debug(f'Getting parameters from file {tests_params_path}')
    if os.path.isfile(tests_params_path):
        with open(tests_params_path) as json_file:
            json_dic = json.load(json_file)

        # Modify relative paths
        shared.replace_string_dict(
            json_dic,
            '<CONFIG_RELATIVE_PATH>',
            os.path.dirname(tests_params_path)
        )

        try:
            test_params = json_dic['tests']
            config_params = json_dic['configurations']
        except KeyError as e:
            logger.error('Error reading parameter test file' + e)
            exit(1)

    else:
        logger.error('Not found file with test parameters.')
        exit(1)

    return test_params, config_params


def set_logger(debug_active=False):

    # Create a custom logger
    logger = logging.getLogger('VALIDATION')
    # Create handlers
    l_handler = logging.StreamHandler()
    # Create formatters and add it to handlers
    l_format = '[%(asctime)s][%(name)s][%(levelname)s] %(message)s'
    l_format = logging.Formatter(l_format)
    l_handler.setFormatter(l_format)
    # Add handlers to the logger
    logger.addHandler(l_handler)
    # Set log level
    if debug_active:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    return logger


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    logger = set_logger(args.debug)


    test_params, config_params = load_test_params(args.params)

    intraprocess = args.intraprocess
    if intraprocess is not None:
        intraprocess = shared.boolean_from_string(intraprocess)

    shm = args.shm
    if shm is not None:
        shm = shared.boolean_from_string(shm)

    result = create_tests(
        test_params,
        config_params,
        args.exe,
        tests=args.test,
        clear=not args.not_remove,
        debug=args.debug,
    )

    val.print_result_bool(
        logger,
        f'Overall test results: ',
        result
    )

    if args.force_remove:
        clear(working_directory())

    if not args.not_remove and result:
        # This clears the DataBases in case the result is correct and remove
        # is on. It is not done for each process because clear_db does not
        # allow to choose which file to clear, so every database will be
        # removed, and there is not an easy way to know the files' names.
        clear(working_directory())

    if result:
        exit(0)
    else:
        exit(1)
