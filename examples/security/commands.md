# COMMANDS

Those have been the commands use to generate this example keys and certificates

## Certification Authority (CA)

```sh
# Generate the Certificate Authority (CA) Private Key
openssl ecparam -name prime256v1 -genkey -noout -out ca.key

# Generate the Certificate Authority Certificate
openssl req -new -x509 -sha256 -key ca.key -out ca.crt -days 3650
```

## DataBroker Certificate

```sh
# Generate the Databroker Certificate Private Key
openssl ecparam -name prime256v1 -genkey -noout -out db.key

# Generate the Databroker Certificate Signing Request
openssl req -new -sha256 -key db.key -out db.csr

# Generate the Databroker Certificate (computed on the CA side)
openssl x509 -req -in db.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out db.crt -days 1000 -sha256

# Generate the Diffie-Hellman (DF) parameters to define how OpenSSL performs de DF key-exchange.
openssl dhparam -out dbparam.pem 2048
```

## Use

```yaml
tls:
  private_key: "db.key"
  password: ""
  dh: "dbparam.pem"
  ca: "sb.crt"
  cert: "ca.crt"
```
