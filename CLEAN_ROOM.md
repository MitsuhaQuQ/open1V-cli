# Clean-room boundary

This source tree is an independent implementation built from an abstract wire
contract and observed input/output behavior. It does not incorporate vendor
firmware, executable code, disassembly, UI assets, strings, or proprietary SDK
headers.

The application defines its own `O1` host-to-bridge protocol. Camera messages
used here are limited to the minimum behavior needed for interoperability and
are represented as numeric wire values. New behavior should be documented as
externally observable behavior before it is implemented here.

Keep recovery images, vendor packages, captures, and exploratory diagnostics
outside this project. Hardware experiments belong in the Arduino repository's
`tests` directory; only reviewed behavior and tests cross this boundary.

