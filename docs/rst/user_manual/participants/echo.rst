.. include:: ../../exports/alias.include

.. _user_manual_participants_echo:

################
Echo Participant
################

This :term:`Participant` prints in ``stdout`` all the discovery information and/or user data that is received by the |ddsrouter|.

In the case of discovery traces, messages such as the following will be displayed:

.. code-block:: bash

    New endpoint discovered: Endpoint{<endpoint_guid>;<endpoint_kind>;<topic>}.

For data reception messages, the traces show the following information:

.. code-block:: bash

    Received data in Participant: <participant_id> in topic: <topic>.

These logs contain the :term:`Participant Name` of the participant that has originally received the message, and the
:term:`Topic` where this message has been received.
Additionally, extra information such as the data :term:`Payload` (in hexadecimal format) and source :term:`Endpoint` :term:`Guid` is displayed in verbose mode:

.. code-block:: bash

    In Endpoint: <endpoint_guid> from Participant: <participant_id> in topic: <topic> payload received: <payload> with specific qos: <specific_qos>.

Notice that this Payload is the same that a standard DDS :term:`DataReader` will receive if it is connected to one
of the Participants of the |ddsrouter|.

.. note::

    This Participant does not perform any discovery or data reception functionality.


Use case
========

Use this Participant in order to see in ``stdout`` the data that is being relayed by the router, as well as information regarding discovery events.
All the data received by any of the Participants of the router will be printed (if ``data`` is ``true``) with its topic and source guid,
along with the payload (in ``verbose`` mode).


Kind aliases
============

* ``echo``

.. _user_manual_participants_echo_configuration:

Configuration
=============

The Echo Participant accepts three different **optional** parameters:

- ``discovery``: Whether to echo information regarding discovery events. Defaults to **true**.
- ``data``: Whether to echo information regarding user data reception. Defaults to **false**.
- ``verbose``: Display detailed information about the user data received (if ``data`` set to ``true``). Defaults to **false**.

Configuration Example
=====================

.. code-block:: yaml

    - name: echo_participant     # Participant Name = echo_participant
      kind: echo
      data: true                 # Print a trace with every arrival of user data
      verbose: true              # Show detailed information on user data reception
      discovery: false           # Do not print traces regarding discovery events
