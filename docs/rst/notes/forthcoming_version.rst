
.. add orphan tag when new info added to this file
.. :orphan:

###################
Forthcoming Version
###################

Next release will include the following **features**:

* New :ref:`Repeater behaviour <use_case_repeater>`,
  a new configuration to create :term:`TURN` servers.
* Timeout argument ``--timeout`` to close the application after the time set has passed.
  Check section :ref:`user_manual_user_interface_timeout_argument` for more information.

Next release will include the following **Fast DDS features**:

* New :term:`Initial Peers` Participant that allows to create a fast WAN communication
  avoiding the Discovery Server configuration.

Next release will include the following **configuration changes**:

* New Configuration Version ``v3.0`` that allow Repeater and initial peers configuration.
* Yaml Validator only validates ``v3.0`` configurations.

Next release will include the following **major changes**:

* ``wan`` Participant Kind uses now Initial Peers Discovery Protocol, while Discovery Server
  requires a new Participant Kind ``wan-discovery-server``.

Next release will include the following **performance improvements**:

* No locking in the :code:`Track` hot-path so transmission is executed faster.
