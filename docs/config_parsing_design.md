# Config Parsing Design

## What this feature does

This change introduces a concrete text format for `drone_config.txt` and a real parser
implementation in `src/config/DroneConfigParser.cpp`.

It also adds `src/config/ConfigParseUtil.h` to centralize line handling shared by
current and upcoming parsers:

- trim leading/trailing whitespace
- strip inline `#` comments
- parse `key = value` lines
- split whitespace tokens (for later `map_input.txt` parsing)

This exists to keep parser files focused on mapping keys to config fields, rather
than duplicating string-cleanup logic in every parser.

## Key design decisions

- **Format shape:** `key = value` lines with optional spaces.
  - Easy to read/edit manually.
  - Stable for deterministic parsing.
- **Comments:** `#` starts an inline comment.
  - Matches common config syntax and improves readability.
- **Recovery strategy:** malformed or unknown lines are ignored.
  - Supports assignment requirement to recover from input issues.
  - Keeps parser robust while error-logging wiring is implemented later.
- **Units in file:** distances are centimeters, angles are degrees.
  - Converted to mp-units strong types at parse time.

## Interfaces to use

- `dmap::parseDroneConfig(path)` in `src/config/DroneConfigParser.h`
  - Input: filesystem path to `drone_config.txt`
  - Output: `DroneConfig`
- `dmap::config_parse::*` helpers in `src/config/ConfigParseUtil.h`
  - Reused by future parser implementations to keep behavior consistent.

## Example file

See `test_data/drone_config.txt` for a complete sample of the current format.
