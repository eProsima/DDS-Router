.. include:: ../../exports/alias.include
.. include:: ../../exports/roles.include

.. _user_manual_participant:

######################
DDS Router Participant
######################

DDS Router :term:`Participant` is a |ddsrouter| entity that works as an interface between a network and
the core of the router.
Participants are the main elements inside the |ddsrouter| functionality.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _user_manual_participant_participant:

Participant
===========

A Participant is an abstraction over the DDS :term:`DomainParticipant`.
This entity manages the dynamic discovery of DDS entities on a specific network or interface.
Each Participant is uniquely identified by a :term:`Participant Name` in a |ddsrouter| execution and has a
predefined :term:`Participant Kind` that specifies the internal general functionality of the Participant.

.. _user_manual_participant_participant_name:

Participant Name
----------------

It is an alphanumeric string that uniquely identifies a Participant in a |ddsrouter| execution.

.. _user_manual_participant_participant_kind:

Participant Kind
----------------

It specifies the kind of the Participant.
There are several Participant kinds already defined, which will specify in general terms how the
Participant behaves.

Participant creation
====================

Each participant configuration is specified as a different item of ``participants`` array, and each of these
configurations has a unique Participant Name that should not be repeated in a |ddsrouter| execution.

Each Participant Kind is associated with one or several names or aliases that represent it.
In order to use a Participant of a specific kind, use ``kind`` tag in the yaml configuration file.
If the kind is not any of the valid aliases, the Participant will not be created and the
execution will fail.

.. note::

    There could be as many Participants as required, and their kinds could be repeated,
    but all names must be unique.

Below are some examples on how to configure a Participant:

.. code-block:: yaml

    - name: participant_1  # New Participant with Name = 'participant_1'
      kind: simple         # 'participant_1' will be created of kind 'simple'
      extra_configuration: ...

.. _user_manual_participant_participant_kinds:

Participant kinds
=================

Below is the list with all the available Participant Kinds.

.. list-table::
    :header-rows: 1

    *   - Participant Kind
        - Aliases
        - Specific |br|
          configuration tags
        - Description

    *   - :ref:`user_manual_participants_echo`
        - ``echo``
        - ``discovery`` |br|
          ``data`` |br|
          ``verbose``
        - Print in `stdout` all user and/or discovery data received.

    *   - :ref:`user_manual_participants_simple`
        - ``simple`` |br|
          ``local``
        - ``domain``
        - Simple DDS DomainParticipant.

    *   - :ref:`user_manual_participants_local_discovery_server`
        - ``discovery-server`` |br|
          ``local-ds`` |br|
          ``ds``
        - ``guid`` |br|
          ``listening-addresses`` |br|
          ``connection-addresses`` |br|
          ``tls``
        - Discovery Server DDS DomainParticipant |br|
          for local communication.

    *   - :ref:`user_manual_participants_discovery_server_wan`
        - ``wan-discovery-server`` |br|
          ``wan-ds``
        - ``guid`` |br|
          ``listening-addresses`` |br|
          ``connection-addresses`` |br|
          ``tls``
        - Discovery Server DDS DomainParticipant |br|
          for WAN communication.

    *   - :ref:`user_manual_participants_wan`
        - ``wan`` |br|
          ``router`` |br|
          ``initial-peers``
        - ``guid`` |br|
          ``listening-addresses`` |br|
          ``connection-addresses`` |br|
          ``tls``
        - Initial Peers DDS DomainParticipant |br|
          for WAN communication.

    *   - :ref:`user_manual_participants_xml`
        - ``xml`` |br|
          ``XML`` |br|
        - ``profile``
        - XML DDS DomainParticipant |br|
          for custom configuration.

..
    This toctree is needed so participants files are linked from somewhere. It is hidden so it is not be visible.

.. toctree::
    :hidden:

    echo
    simple
    local_discovery_server
    wan_discovery_server
    wan
    xml
