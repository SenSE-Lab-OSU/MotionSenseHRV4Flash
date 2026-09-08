# ECGv0 history rewrite map

## Recorded baseline

- Main ref after fetch: `b61971b60e08a31055aec3a7ae9edd5b0b1cb7ec`
- Original ECG ref after fetch: `070793f51ce01262724812d73e3ed47b3c2a6bd9`
- Common merge base: `68967d2d31b7ca308877b0c9e644ce5e0feb2a27`
- Local archival safety ref: `refs/archive/ecgv0-original-2026-08-27` -> `070793f51ce01262724812d73e3ed47b3c2a6bd9`
- Rewrite branch: `codex/ecgv0-history-cleanup`
- Rewritten tip: `7aac128a5b2c2e028aebd6ea5b076680cc1cbc4b`

The source range contains 41 commits, including one historical merge. The
rewrite contains 11 linear commits. Every source-side commit is represented
below. `eaccc6b` is deliberately split: its recording/transport changes are in
the recording commit, while its final button electrical adjustment stays with
the ECGv0-only ship-mode commit.

| Rewritten commit | Original commits represented | Rationale |
| --- | --- | --- |
| `fced74aa7c55221e0806a5151f439da4f0868f26` `refactor(ecgv0): remove legacy processing and simplify sensor orchestration` | `b42b2b02934d032c9ae31bdc5b46328e0bb5864a`, `09cb3f893269cd747a96ca52b03238c34b5cce31`, `d27796573b63ab0bfca8e055a0dbf68748e43316`, `4465f61570159b71a621f64cd5a0006fe7baafaa` | One coherent removal of obsolete ML, magnetometer/orientation, and legacy orchestration paths. |
| `1dc6d8d5c5941fd7c84cb0aac5d451695b2a9722` `feat(ecgv0): add MAX30001 ECG hardware support` | `13a5980464b095ce2ce068725d343954d1cb24a7`, `2f934a5246567b870a80eeaac81232d891d827ed`, `d068c5c37502e40c932b07c44b50393d5c6a7851`, `0805ef86a0055929246da73b69bfcf95c8495827`, `a957c4eb93eb49e8587e79f0fb8541d607e3d73c`, `d160fb7aba8db3ba8cd509bd2c8fb4fe78a520d0` | Groups the MAX30001 binding, probe, acquisition, and intended filter configuration. |
| `ac14f318049c9bc57f000accb23cb743d7ac71dd` `feat(ecgv0): integrate ECG recording, USB diagnostics, and BLE transport` | `2df5c01676dfa0e93e81a70967f446965624172e`, `53f4f7e47ef90f5f06d8a4a7551ca68bc3cc6185`, `eaccc6bdce6db48517fe52480042001e04b099b6` (recording/USB/BLE hunks), `81680eeddf17fd8254d6b5b80d167d5a95c412a6`, `a3af46ed773983bc87848ca520efb9df83ce1321`, `06fb1206e67ea9207733366067830b84378b5068`, `721faa4e10b030a71ed32ff976c371f5193d4ea3` | Keeps the persisted ECG record flow, mass-storage diagnostics, BLE counter/stream behavior, documentation, and related BLE preference together. |
| `10a7612edff0c246433a67171483c172ff722892` `build(ecgv0): standardize NCS 2.9.3 clock, sysbuild, and developer tooling` | `0fbcf5357ce91154b9a84fecedf3ddbc60b09e99`, `0203ca65b2d5709c814155b409e3f243aa1ded52`, `012940ae3539fda6052baa7a5cccd9d8ce04ee8d`, `ce2b54379c249f28de5fbe15fcf678386c2b039e`; historical merge `1db2cacd7390ac7733bf64d9a700a1d7e39366f0` | The merge topology is flattened; its two parent-side changes are represented by this commit and the preceding recording/transport commit. |
| `3c53c732f6938eb193d77500f65cc738d94952e3` `feat(ecgv0): add the ECG-specific ICM-20948 implementation` | `e7b543e58000cf3bac6e9f51d958549571b32e9b` | Preserves the intentionally separate ECGv0 IMU driver, record types, and focused tests. |
| `8f92dad57047fb96e720289167310c7fb0a08f87` `feat(ecgv0): anchor accelerometer and ECG records to RTC timing` | `43455e9e73aad66867fd6d50e9e43a0521b71527`, `d24fe1d2512d8463855e1aff50ff06e3805093fc`, `a8f6c23a168ecc65695c471c082253421edc99b2` | Keeps RTC0 handling, FSYNC timing, FIFO watermark, timestamped record formats, and tests together. |
| `5700755183f84bdf85aefb030760917cc2da2f5a` `feat(ecgv0): add stable device identity and Git build provenance` | `8e632851a3a1395b9f5c9e507a11d3d3f09fe8c0` | Identity/provenance mechanism only; its ship-mode context is represented by the later ship-mode commit. |
| `9919ee8e5652e67bc4ef7699ab39b2b7136d5046` `feat(ecgv0): configure the 200 mAh battery and restore battery monitoring` | `a1f0c91ed687f93a2f8f2e2948aac076529f0c4d`, `f56d4404419e8432bc60aa248826b6dd37af2db9` | Retains the confirmed 200 mAh gauge and battery-monitor behavior. |
| `abfd04b4b977cb14668dafa5845160699ce68e62` `feat(ecgv0): add button-driven ship mode while preserving USB storage access` | `702fd4601c552537bbaca222991dbb203fb62716`, `5542eb86f8f3df5e906449e77594b383f46d71d0`, `912f812baaa5110500cb2253c1d63e80354f62d8`, `eaccc6bdce6db48517fe52480042001e04b099b6` (button electrical hunk) | Groups ECGv0-only physical-button facts, ship-mode behavior, and the USB mass-storage fix. |
| `244adccbed1d53bd8dc2d75128eebfaaf2ca983b` `fix(storage): preserve NAND linkage and validate ECC and 8-Gbit geometry` | `43d07fd9e41c8eadf448b7f025031c10c25f0ad4`, `dc7f29bc5197b9bc52858cb5899c5da4744c56db`, `5718d2ed872d7e45ecb63295b628d76dd132cfd3` | Keeps LTO linkage, ECC rejection/diagnostics, and two-device 8-Gbit geometry in one storage change. |
| `7aac128a5b2c2e028aebd6ea5b076680cc1cbc4b` `feat(logging): persist reset and filesystem logs without disrupting collection` | `9de2f2deb513b092e63bd5875402e0935df3e1b8`, `90c6e283c21bba43e443c139afa8701ee87ece46`, `24ea8c73d2cd526b2be1153baa10b553f05ac6a0`, `6b30943dfea9b744d56621d3f144c43819c183b6`, `5acb4910f7c16c3ad22f6e1b7c57097126cdd556`, `070793f51ce01262724812d73e3ed47b3c2a6bd9` | Groups NAND logging, reset-cause capture, collection-time USB CDC suppression, and newline/message cleanup. |

No main-only commit is represented by this map. The rewrite’s exact tree
identity with the archived ECG tip is the definitive check that no source file,
mode, binary, deletion, generated file, or directory rename was introduced.
