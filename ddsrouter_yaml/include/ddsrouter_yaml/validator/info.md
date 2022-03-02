# YAML VALIDATOR

## Work In Progress

The method followed to create an schema is the following:

1. For each small subpart of the yaml
    1. Create yaml valid for this part *[Result in `<part>.yaml`]*
    1. Transform it into json *[Result in `<part>.json`]*
    1. Create json schema from this json and customizing values and conditions *[Result in `<part>_validator.json`]*
1. Join all validators into one

**ACTUAL STATE**: create subparts `<part>_validator.json`

---

## Useful links

- Convert YAML to JSON online tool:
<https://onlineyamltools.com/convert-yaml-to-json>

- Get JSON schema from JSON online tool:
<https://jsonformatter.org/json-to-jsonschema>

- Check JSON validity against a schema  online tool:
<https://www.jsonschemavalidator.net/>

- Conditional in JSON schema
<https://json-schema.org/understanding-json-schema/reference/conditionals.html>
