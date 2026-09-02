# Known Issues

## Large relay allocation on the Bluetooth RX stack

The NUS notification path currently calls `relay_enqueue()`, which creates a
roughly 520-byte `struct relay_message` as a local variable. Because GATT
notification callbacks run in the Bluetooth RX thread, this allocation consumes
more than half of that thread's configured stack.

This causes a reproducible UsageFault on the nRF54L15 build:

- `CONFIG_BT_RX_STACK_SIZE=1024`
- `relay_enqueue()` reserves 520 bytes in its function prologue
- At the fault, `PSP` was equal to `PSPLIM`, confirming stack exhaustion
- The captured call path was `nus_notification()` -> `relay_enqueue()` ->
  `k_msgq_put()`

The nRF5340 build has not reproduced the fault because its Bluetooth RX stack is
larger (`CONFIG_BT_RX_STACK_SIZE=1200`) and currently has just enough headroom.
This is still a latent defect on nRF5340; changes to the SDK, compiler, logging,
or callback depth could expose it there as well.

### Future fix

Move relay-message storage out of the Bluetooth callback stack. Use a bounded
static `k_mem_slab` for relay messages and queue only pointers to allocated
blocks. Keep allocation and queue operations nonblocking, release blocks on all
success and failure paths, and preserve notification parsing before relay
enqueueing.

Increasing `CONFIG_BT_RX_STACK_SIZE` alone is only a workaround and should not
be treated as the final fix. Remove or update this entry after the slab-based
implementation passes pristine builds and hardware tests on both nRF54L15 DK
and nRF5340 DK.
