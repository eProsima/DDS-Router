# DDS ROUTER SECURITY BACKDOOR TEST

This DDS-Router test belongs to the docker tests that are executed using a docker-compose application that rises several DDS Participants and DDS Routers.
This test case check the connection between a secured and a non secured DDS Domain.

## Test scenario

In this test case, there are 2 DDS Domains and 1 DDS Router:

- **A**: DomainParticipants belong to DDS Domain 0.
  - Entities:
    - DomainParticipant: DataWriter [topic_0]
    - DomainParticipant: DataReader [topic_1]
    - DomainParticipant: DataWriter [topic_2]
    - DomainParticipant: DataReader [topic_2]
    - DomainParticipant: DataReader [topic_3]
    - DomainParticipant: DataWriter [topic_4]
    - DomainParticipant: DataReader [topic_4]
    - DomainParticipant: DataWriter [topic_5]
    - DomainParticipant: DataReader [topic_5]
  - Executable:
    - `AdvancedConfigurationExample`

- **S**: DomainParticipants belong to DDS Domain 0 and uses DDS Security with authentication and encryption.
  - Entities:
    - DomainParticipant: DataReader [topic_0]
    - DomainParticipant: DataWriter [topic_1]
    - DomainParticipant: DataReader [topic_2]
    - DomainParticipant: DataWriter [topic_3]
    - DomainParticipant: DataReader [topic_3]
    - DomainParticipant: DataWriter [topic_4]
    - DomainParticipant: DataReader [topic_4]
    - DomainParticipant: DataWriter [topic_5]
    - DomainParticipant: DataReader [topic_5]
  - Configuration
    - XML:
      - `configurations/secure_participant_configuration.xml`
      - profile name: `xml_secure_participant_backdoor_test`
  - Executable:
    - `AdvancedConfigurationExample`

- **R**: DDS Router that connect both participants
  - Entities:
    - Participant `local`
    - Participant `xml`
  - Configuration:
    - XML:
      - `configurations/router_configuration.xml`
      - profile name: `xml_secure_router_backdoor_test`
    - yaml:
      - `ddsrouter.yaml`
  - Executable:
    - `ddsrouter`

### Topics

All topics are best-effort volatile.

DDS Router is not allowed in topic `topic_2` and `topic_3`.
DDS Router is not allowed to publish in `topic_4`.
DDS Router is not allowed to subscribe in `topic_5`.

Thus, the expected behaviour is:

- *A* DataReader [topic_1]: Read everything
- *A* DataReader [topic_2]: Read everything
- *A* DataReader [topic_3]: Read nothing
- *A* DataReader [topic_4]: Read both domains
- *A* DataReader [topic_5]: Read only from *A*
- *S* DataReader [topic_0]: Read everything
- *S* DataReader [topic_2]: Read nothing
- *S* DataReader [topic_3]: Read everything
- *S* DataReader [topic_4]: Read only from *S*
- *S* DataReader [topic_5]: Read everything
