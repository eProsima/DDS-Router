.. include:: ../../exports/alias.include

.. _user_manual_participants_xml:

###############
XML Participant
###############

This type of :term:`Participant` refers to a :term:`DomainParticipant` that uses QoS profiles loaded from XML files to be configured.

|fastdds| supports XML to fully configure a DomainParticipant.
Using XML configuration, user have whole access to the full configuration of a DDS DomainParticipant.
Check following `documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/domainparticipant.html>` for further information about configuring a DDS DomainParticipant with XML.
For further information regarding how to load XML configuration files to |ddsrouter| check following section :ref:`user_manual_configuration_load_xml`.

.. note::

    This kind of Participant is meant for advanced users as XML profiles will overwrite the default internal settings of the DDS Router.


Use case
========

Use this Participant to fully configure a DomainParticipant, its discovery methods, transport options, DDS QoS, etc.
Main use case for this Participant is using **DDS Security**, that requires XML configuration from user side.

.. TODO

    Add link to security documentation


Kind aliases
============

* ``xml``
* ``XML``


Configuration
=============

XML Participant allows configure a profile name.
Such name will be used as QoS profile when creating the internal DomainParticipant.
Whole DomainParticipant configuration settings must be configured via XML, |ddsrouter| will not configure any attribute or QoS for it (that is why this participant is not advisable for non expert users).

* To configure the profile, check the following section
  :ref:`Configuration section <user_manual_configuration_profile>`.


Configuration Example
=====================

Configure a XML Participant that uses a profile called `custom_participant_configuration` loaded previously from an XML configuration.
How to load such XML profiles is explained in `following documentation <user_manual_configuration_load_xml>`.

.. code-block:: yaml

    - name: xml_participant                       # Participant Name = xml_participant

      kind: xml

      profile: custom_participant_configuration   # Configure participant with this profile
