# ECG → Bridge ESB demo format

Agreed demo contract, 2026-10-27. Target SDK: project-managed NCS 2.9.3
(`C:\ncs\SenSEv2.9.3`), nRF5340. This document specifies the intended TX/RX
configuration; it does not mean the current Bridge firmware already implements it.

## Radio settings (both ends)

| Setting | Value |
| --- | --- |
| Roles | ECG PTX; Bridge PRX, continuously listening when available |
| Channel | `40` via `esb_set_rf_channel(40)` = 2440 MHz |
| Bitrate | `ESB_BITRATE_2MBPS` |
| Framing | `ESB_PROTOCOL_ESB_DPL` |
| Application payload | Exactly 4 bytes; reject other lengths |
| CRC | `ESB_CRC_16BIT`, standard SDK ESB CRC configuration |
| Address length | 5 bytes via `esb_set_address_length(5)` |
| Pipe | Pipe 0 only; `esb_enable_pipes(0x01)` |
| Base address 0 | `{ 0xE7, 0xE7, 0xE7, 0xE7 }` via `esb_set_base_address_0()` |
| Prefix | Pipe 0 prefix `0xE7` via `esb_set_prefixes(prefixes, 1)` |
| Selective auto ACK | `selective_auto_ack = true` on both ends |
| TX payload flags | `pipe = 0`, `length = 4`, `noack = true` |
| TX power | `ESB_TX_POWER_0DBM` |
| Retransmissions | None; set `retransmit_count = 0` |

This is a broadcast convention using one shared ESB address. There is no
destination or sender ID in the payload. ESB still requires the radio address.
Use one ECG transmitter for an unambiguous demo.

DPL is required for the per-packet no-ACK behavior in this SDK; the application
length remains fixed at four bytes. Do not select `ESB_PROTOCOL_ESB` fixed-length
framing. Let the ESB library generate its header, PID, length, and CRC. Bridge
must not queue the sample's demonstration ACK payload.

## Four-byte timestamp

Bytes 0–3 contain one unsigned 32-bit integer in **little-endian** order:
the ECG application's existing LFCLK-derived RTC0 counter in **512 Hz ticks**.
One tick is 1/512 second (1.953125 ms), not one raw 32768 Hz LFCLK cycle.

- Read `rtc0_collection_counter_get(&ticks)`, declared in
  `ECGv0/src/BLEService.h` and implemented in `ECGv0/src/BLEService.c`.
- Reuse its existing initialization and overflow extension. Do not clear,
  reconfigure, or replace the RTC for ESB. If the read fails, skip the transmission.
- Sample as close as practical to handing the payload to the network-core TX
  path. This is the **ECG timestamp at payload preparation**, not a captured
  radio transmission timestamp or the Bridge's reception time.
- Serialize with `sys_put_le32(ticks, payload.data)`; decode on Bridge with
  `sys_get_le32(payload.data)` after checking `payload.length == 4`.
- Epoch is ECG RTC0 initialization at boot. Reboot resets it; it is not UTC and
  is not synchronized to Bridge. It continues across collection stop/start.
- The software-extended value wraps modulo 2^32 ticks, every 8,388,608 seconds
  (about 97.09 days). Do not transmit only the raw 24-bit hardware counter.

Example: timestamp `512` (one second after counter initialization) is transmitted
as `00 02 00 00`. There are no other application bytes, version fields, sequence
numbers, or checksums.

## Minimal transmitter and receiver behavior

ECG transmits opportunistically; no fixed interval is required by this format.
Use at most one outstanding packet and skip an opportunity when busy or when
radio access is unavailable. No application backlog, retries, backoff, collision
handling, or delivery guarantee. Normal ESB internal FIFOs are still used; they
are not a request to add application buffering. TX completion does not prove RX.

Bridge starts PRX and drains the ESB RX FIFO on receive events, accepting only
pipe 0, length 4. It remains listening, sends no ACK for these no-ACK packets,
and needs no application queue. Counters, diagnostics, and USB forwarding are
separate implementation steps. Repeated timestamps are possible, and this format
cannot identify packet loss, distinguish senders, or reliably detect reboot.

On nRF5340 the ESB radio runs on the network core. ECG already uses BLE: the ECG
agent must coordinate ESB access with the existing BLE controller using a
supported radio-timeslot integration. Do not let ESB take ownership of RADIO
while BLE owns it. Transfer the application-core RTC timestamp to the network
core; do not substitute the network core's local clock. Scheduling and IPC are
implementation choices and do not change this wire format.

## Implementation references

- Nordic [ESB packet transactions](https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/protocols/esb/index.html#packet-transaction)
  describes selective ACK and no-ACK operation. This link tracks latest docs;
  the local SDK below is the implementation authority for this project.
- NCS 2.9.3: `nrf/include/esb.h` and `nrf/subsys/esb/esb.c` under the managed SDK.
  In this release, the fixed-framing TX path waits for ACK; the DPL path honors
  `selective_auto_ack` and the public payload `noack` flag.
- Bridge receiver to update: `bridge/src/main.c`. Set the channel and pipe mask
  explicitly, remove the queued sample ACK payload, and validate/decode the
  four-byte timestamp. Existing sample defaults alone do not implement this contract.
