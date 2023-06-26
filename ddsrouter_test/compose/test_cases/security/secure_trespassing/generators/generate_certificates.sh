#!/bin/bash
# Script to autogenerate all certificates for DDS Router Security Backdoor Test
# This script is meant to be executed from <ddsrouter_path>/ddsrouter_test/compose/test_cases/security/backdoor/generators/ (its location in the repository)

# Directory to store the files that are actually needed in test
CERTS_WORKSPACE="$(pwd)/../certs"
# Directory to store the files to configure this commands
CONF_WORKSPACE="$(pwd)/"
# Directory to store result files that are not required in test
EXTRA_WORKSPACE="$(pwd)/../extra"

mkdir -p ${CERTS_WORKSPACE}
mkdir -p ${CONF_WORKSPACE}
mkdir -p ${EXTRA_WORKSPACE}
touch ${EXTRA_WORKSPACE}/index.txt

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -x509 -days 3650 -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -keyout ${EXTRA_WORKSPACE}/maincakey.pem -out ${EXTRA_WORKSPACE}/maincacert.pem -config ${CONF_WORKSPACE}/maincaconf.cnf

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/localconf.cnf -keyout ${EXTRA_WORKSPACE}/localkey.pem -out ${EXTRA_WORKSPACE}/localreq.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/localreq.pem -out ${EXTRA_WORKSPACE}/localcert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/routerconf.cnf -keyout ${EXTRA_WORKSPACE}/routerkey.pem -out ${EXTRA_WORKSPACE}/routerreq.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/routerreq.pem -out ${EXTRA_WORKSPACE}/routercert.pem

openssl smime -sign -in governance.xml -text -out ${EXTRA_WORKSPACE}/governance.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

openssl smime -sign -in permissions.xml -text -out ${EXTRA_WORKSPACE}/permissions.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

#########
# Copy result files required in CERTS directory

file_list=(
    "maincacert.pem"
    "localkey.pem"
    "localcert.pem"
    "routerkey.pem"
    "routercert.pem"
    "governance.smime"
    "permissions.smime"
)

for file in "${file_list[@]}"
do
    cp "${EXTRA_WORKSPACE}/${file}" "${CERTS_WORKSPACE}"
done
