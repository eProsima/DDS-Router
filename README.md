# DATABROKER PROTOTYPE

The Fast DDS Databroker is an application developed by eProsima that allows, using Fast DDS,
to communicate by DDS protocol different networks.

## Installation

### Requirements

#### Fast DDS

#### Yaml

## Execution

### Parameters

Each argument represented below follows the pattern `short option / large option (default value) [format example]`
Those without default value are flags and their default value is _non active_ or `false`.

- `-h / --help`
- `-s / --server-id (0)`
- `-g / --server-guid (0)`
- `-l / --listening-addresses ("127.0.0.1,11800") ["127.0.0.1,11800;192.168.1.1,11801"]`
- `-c / --connection-addresses ("") ["127.0.0.1,11800,0;192.168.1.1,11801,1"]`
- `-i / --interactive`
- `-t / --time (0)`
- `-d / --domain (0)`
- `-r / --ros`
- `-u / --udp`
- `-w / --whitelist ("") ["HelloWorld;chatter"]`
- default configuration file: `**/DATABROKER_CONFIGURATION.yaml` if exists

### Commands

- `h / help`
- `a / add <topic name>`
- `r / rm / remove <topic name>`
- `l / load [<file path>]`
- `e / x / exit`
