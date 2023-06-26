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

# CA
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -x509 -days 3650 -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -keyout ${EXTRA_WORKSPACE}/maincakey.pem -out ${EXTRA_WORKSPACE}/maincacert.pem -config ${CONF_WORKSPACE}/maincaconf.cnf

# Local 0
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local0conf.cnf -keyout ${EXTRA_WORKSPACE}/local0key.pem -out ${EXTRA_WORKSPACE}/local0req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local0req.pem -out ${EXTRA_WORKSPACE}/local0cert.pem

# Local 1
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local1conf.cnf -keyout ${EXTRA_WORKSPACE}/local1key.pem -out ${EXTRA_WORKSPACE}/local1req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local1req.pem -out ${EXTRA_WORKSPACE}/local1cert.pem

# Router Local 0
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/routerlocal0conf.cnf -keyout ${EXTRA_WORKSPACE}/routerlocal0key.pem -out ${EXTRA_WORKSPACE}/routerlocal0req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/routerlocal0req.pem -out ${EXTRA_WORKSPACE}/routerlocal0cert.pem

# Router Local 1
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/routerlocal1conf.cnf -keyout ${EXTRA_WORKSPACE}/routerlocal1key.pem -out ${EXTRA_WORKSPACE}/routerlocal1req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/routerlocal1req.pem -out ${EXTRA_WORKSPACE}/routerlocal1cert.pem

# Server
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/serverconf.cnf -keyout ${EXTRA_WORKSPACE}/serverkey.pem -out ${EXTRA_WORKSPACE}/serverreq.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/serverreq.pem -out ${EXTRA_WORKSPACE}/servercert.pem

# Client
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/clientconf.cnf -keyout ${EXTRA_WORKSPACE}/clientkey.pem -out ${EXTRA_WORKSPACE}/clientreq.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/maincaconf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/clientreq.pem -out ${EXTRA_WORKSPACE}/clientcert.pem

# Governance & Permissions
openssl smime -sign -in governance.xml -text -out ${EXTRA_WORKSPACE}/governance.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

openssl smime -sign -in permissions_lan0.xml -text -out ${EXTRA_WORKSPACE}/permissions_lan0.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

openssl smime -sign -in permissions_lan1.xml -text -out ${EXTRA_WORKSPACE}/permissions_lan1.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

openssl smime -sign -in permissions_wan.xml -text -out ${EXTRA_WORKSPACE}/permissions_wan.smime -signer ${EXTRA_WORKSPACE}/maincacert.pem -inkey ${EXTRA_WORKSPACE}/maincakey.pem

#########
# Copy result files required in CERTS directory

file_list=(
    "maincacert.pem"

    "local0key.pem"
    "local0cert.pem"

    "routerlocal0key.pem"
    "routerlocal0cert.pem"

    "serverkey.pem"
    "servercert.pem"

    "clientkey.pem"
    "clientcert.pem"

    "routerlocal1key.pem"
    "routerlocal1cert.pem"

    "local1key.pem"
    "local1cert.pem"

    "governance.smime"
    "permissions_lan0.smime"
    "permissions_lan1.smime"
    "permissions_wan.smime"
)

for file in "${file_list[@]}"
do
    cp "${EXTRA_WORKSPACE}/${file}" "${CERTS_WORKSPACE}"
done
