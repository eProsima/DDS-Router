# COMMANDS

Those have been the commands use to generate this example keys and certificates

## Certification Authority (CA)

```sh
# Generate the Certificate Authority (CA) Private Key > ca.key
openssl ecparam -name prime256v1 -genkey -noout -out ca.key
# openssl ecparam -name prime256v1 -genkey | openssl ec -aes256 -out ca.key -passout pass:cakey # with password

# Generate the Certificate Authority Certificate > ca.crt
openssl req -new -x509 -sha256 -key ca.key -out ca.crt -days 3650 -config ca.cnf
# openssl req -new -x509 -sha256 -key ca.key -out ca.crt -days 3650 -config ca.cnf -passin pass:cakey # with password
```

## DataBroker Certificate

```sh
# Generate the Databroker Certificate Private Key > databroker.key
openssl ecparam -name prime256v1 -genkey -noout -out databroker.key
# openssl ecparam -name prime256v1 -genkey | openssl ec -aes256 -out databroker.key -passout pass:dbpass # with password

# Generate the Databroker Certificate Signing Request  > databroker.csr
openssl req -new -sha256 -key databroker.key -out databroker.csr -config databroker.cnf
# openssl req -new -sha256 -key databroker.key -out databroker.csr -config databroker.cnf -passin pass:dbpass # with password

# Generate the Databroker Certificate (computed on the CA side) > databroker.crt
openssl x509 -req -in databroker.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out databroker.crt -days 1000 -sha256
# openssl x509 -req -in databroker.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out databroker.crt -days 1000 -sha256 -passin pass:cakey # with password
```

## DH PARAMETERS

```sh
# Generate the Diffie-Hellman (DF) parameters to define how OpenSSL performs de DF key-exchange > db_params.pem
openssl dhparam -out db_params.pem 2048
```

## Use

```yaml
tls:
  private_key: "databroker.key"
  ca: "databroker.crt"
  cert: "ca.crt"
  dh_params: "db_params.pem"
  password: ""
```
