# MANUAL TEST

In order to check the correct functionality of the DDS Router manually, execute the following tests:

## Test Descriptions

The tests that are described below are executed by following commands specified in [this section](#commands).
Each line of each tests refer to an execution in a specific terminal.
The number between brackets `[]` is the reference to the command to execute in which order:
    i.e. `[1.0]` means: execute `.install/ddsrouter/ddsrouter -h`
Source commands (those starting in `[0.`) are not included in examples to keep them more readable.

Consecutive command lines means that commands must be executed in different terminals.
Those that have spaces in between, means that must wait for other terminal commands to be executed.

Most of these examples are checked by getting the output in stdout.
The end of the line after `->` is the expected info in stdout.
End every test that not specified a failure with commands `[4.0][4.1]` to check that the execution was correct.

Move file <./resources/DDS_ROUTER_CONFIGURATION.yaml> to the workspace in order to use it as default file
Move directory <./resources> to the workspace in order to get an easy access to yaml configurations.

## Test Cases

- User Interface Positive Cases
  - [ ] User Interface Help
  - [ ] User Interface Std execution with default file
  - [ ] User Interface Std execution with echo example file

- User Interface Negative Cases
  - [ ] User Interface No file set
  - [ ] User Interface file set wrongly

- Examples Cases
  - [ ] Echo Example


## User Interface Testing

### Positive Cases

#### User Interface Help

```txt
[1.0]           -> Help information
```

#### User Interface Std execution with default file

```txt
[1.1]           -> Data from Echo Participant
[3.0]           -> Data published
```

#### User Interface Std execution with echo example file

```txt
[1.2]           -> Data from Echo Participant
[3.0]           -> Data published
```

### Negative Cases

## Example Testing

These executions must be run with a clean installation.
It test every example given in the Documentation.
Each of it requires a different kind of talker and listener, and may require a Fast DDS CLI Discovery Server.

TODO Add examples from resources

### Echo Example

```txt
[1.2]           -> Data from Echo Participant
[3.0]           -> Data published
```

## Commands

```sh
# [0.0]
# Fast DDS Source <from fastdds installation path>
source install/setup.bash

# [0.1]
# DDS Router Source <from ddsrouter installation path>
source install/setup.bash

# [0.2]
# ROS 2 Source <from ros2 installation path>
source install/setup.bash
```

```sh
# [1.0]
# DDS Router Help
.install/ddsrouter/ddsrouter -h

# [1.1]
# DDS Router Default Config file
.install/ddsrouter/ddsrouter

# [1.2]
# DDS Router Default Config file
.install/ddsrouter/ddsrouter -c resources/echo.yaml
```

```sh
# [2.0]
# Std listener
ros2 run demo_nodes_cpp listener

# [2.1]
# Domain 1 listener
ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

# [2.2]
# Discovery Server Client listener
ROS_DISCOVERY_SERVER=";127.0.0.1:11888" ros2 run demo_nodes_cpp listener
```

```sh
# [3.0]
# Std talker
ros2 run demo_nodes_cpp talker

# [3.1]
# Domain 1 talker
ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker

# [3.2]
# Discovery Server Client talker
ROS_DISCOVERY_SERVER=";127.0.0.1:11888" ros2 run demo_nodes_cpp talker
```

```sh
# [4.0]
# Execute a Fast DDS Discovery Server
fastdds discovery -i 1 -p 11888 -i "127.0.0.1"
```

```sh
# [5.0]
# Exit execution
^C

# [5.1]
# Check exit code status of the last process
echo $?
```

## WAN Testing

TODO
