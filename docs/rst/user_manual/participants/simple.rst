.. include:: ../../exports/alias.include

.. _user_manual_participants_simple:

##################
Simple Participant
##################

This kind of :term:`Participant` refers to a Simple DDS :term:`DomainParticipant`.
This Participant will discover all Participants deployed in its own local network in the same domain via multicast
communication, and will communicate with those that share publication or subscription topics.


Use case
========

Use this Participant in order to communicate an internal standard DDS network, such as a ROS 2 or Fast DDS network
in the same LAN.


Kind aliases
============

* ``simple``
* ``local``


Configuration
=============

The main configuration of a Simple Participant is the :term:`Domain Id` on which it will listen for DDS
communications, which is ``0`` if it is not set.
Check :ref:`Configuration section <user_manual_configuration_domain_id>` for further details.

It also accepts the :ref:`ignore-participant-flags <user_manual_configuration_ignore_participant_flags>`,
:ref:`transport <user_manual_configuration_custom_transport_descriptors>`,
:ref:`ros2-easy-mode <user_manual_configuration_easy_mode>`,
:ref:`whitelist-interfaces <user_manual_configuration_interface_whitelist>` and
:ref:`qos <user_manual_configuration_participant_topic_qos>` tags.


Configuration Example
=====================

.. code-block:: yaml

    - name: simple_participant     # Participant Name = simple_participant
      kind: simple
      domain: 2                    # Domain Id = 2
