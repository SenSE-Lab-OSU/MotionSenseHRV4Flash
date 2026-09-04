# Known issues and limits

## No hardware DFU validation in this source tree

The DFU wire parser, host CLI, and both DK firmware targets have automated
build/test coverage, but a complete upload/test/reset/reconnect/confirm cycle
still requires an approved hardware session with a compatible SMP + MCUboot
peripheral. Do not infer target interoperability from a successful compile.

## Signature validation remains peripheral policy

The host validates MCUboot BIN structure and SHA-256 identity metadata only.
It detects signature TLVs but intentionally does not require a public key or
cryptographically validate one. With v1's default BLE security disabled, the
peripheral MCUboot configuration remains the authority deciding whether a
candidate image is acceptable. See `OPERATIONS.md` before enabling a signed
image or paired-link production policy.

## VCOM physical ownership is host-visible only through protocol events

The DK UARTE cannot observe interface-MCU DTR directly. Keep both VCOMs open
with DTR asserted and treat `RELAY_IDLE` as the authoritative NUS-to-DFU handoff
event. A failed async RX shutdown deliberately leaves the binary port
unavailable rather than risking overlapping NUS TX and DFU RX.
