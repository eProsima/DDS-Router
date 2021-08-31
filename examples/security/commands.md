# COMMANDS

Those have been the commands use to generate this example keys and certificates

## Certification Authority

```sh
openssl ecparam -name prime256v1 -genkey -noout -out ca.key
openssl req -new -x509 -sha256 -key ca.key -out ca.crt -days 3650
# ES .
```

## DataBroker Certificate

```sh
openssl ecparam -name prime256v1 -genkey -noout -out db.key
openssl req -new -sha256 -key db.key -out db.csr
# ES .
openssl x509 -req -in db.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out db.crt -days 1000 -sha256
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
