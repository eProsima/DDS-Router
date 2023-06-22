# GENERATE DDS SECURITY CERTIFICATES

In order to use DDS Router with DDS Security, user must generate the DDS Router certificates allowing one or more of its participants to use Security in DDS layer.
In order to do so, check the following [documentation](https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html) regarding DDS Security.

Here there are some generators that exemplifies the generation of real certificates.
This will generate the certificates associated with a DDS Router execution configured with files in `configuration` dir.

```sh
# from resources/configurations/security/dds/generators/generate_certificates.sh
./generate_certificates.sh
```
