
.. add orphan tag when new info added to this file
.. :orphan:

###################
Forthcoming Version
###################

Next release will include the following **major features**:

* New **QoS Transparency Module**.

Next release will include the following **features**:

* New :ref:`Repeater behaviour <use_case_repeater>`,
  a new configuration to create :term:`TURN` servers.
* Add RPC support (compatibility with *ROS 2* services).
* Timeout argument ``--timeout`` to close the application after the time set has passed.
  Check section :ref:`user_manual_user_interface_timeout_argument` for more information.
* Add TLS-SNI support.
* Add **external port** support so internal and external port in network router configuration could be different.

Next release will include the following **DDS features**:

* New :term:`Initial Peers` Participant that allows to create a fast WAN communication
  avoiding the Discovery Server configuration.
* Participants names are set from ParticipantIds.

Next release will include the following **configuration changes**:

* New Configuration Version ``v3.0`` that allow Repeater and initial peers configuration.
* Yaml Validator only validates ``v3.0`` configurations.
* Address supports an :code:`external-port` value to set this in a listening address.

Next release will include the following **major changes**:

* ``wan`` Participant Kind uses now Initial Peers Discovery Protocol, while Discovery Server
  requires a new Participant Kind ``wan-discovery-server``.

Next release will include the following **performance improvements**:

* No locking in the :code:`Track` hot-path.
