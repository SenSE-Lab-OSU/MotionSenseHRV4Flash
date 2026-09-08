# Phase 0 — consolidation provenance and import matrix

Recorded: 2026-08-27

## Refreshed sources

`git fetch origin --prune` was run before the import work.

| Source | Ref / commit | Role |
| --- | --- | --- |
| PPGv2 source | `origin/main` at `ae58cb6f4e63dcca6f5b3fe1ed99103c0078d654` | Refreshed main application source |
| ECGv0 source | `codex/ecgv0-history-cleanup` at `7aac128a5b2c2e028aebd6ea5b076680cc1cbc4b` | Accepted 11-commit ECG rewrite baseline |
| Original ECG archive | `refs/archive/ecgv0-original-2026-08-27` at `070793f51ce01262724812d73e3ed47b3c2a6bd9` | Immutable local archival source |
| Common merge base | `68967d2d31b7ca308877b0c9e644ce5e0feb2a27` | Common ancestor of the maintained lines |
| Rewrite documentation | `codex/ecgv0-history-docs` at `f5a87437a92cf39e697b3b82ebac63d827b38d65` | Accepted Phase -1 audit documentation only |

The accepted ECG baseline and archival ECG source both resolve to tree
`fd0b7d850237693ea42918314d61fe3598a09a1b`.

At import time, `origin/main...codex/ecgv0-history-cleanup` contained 24
main-side commits and 11 cleaned ECG-side commits. Relative to the original
ECG archival tip, the divergence was 24 main-side commits and 41 original
ECG-side commits. The complete original-to-cleaned ECG mapping is in
`docs/history/ecgv0-history-rewrite-map.md`.

## Import matrix

| Source scope | Destination | Phase 0 disposition | Evidence |
| --- | --- | --- | --- |
| `7aac128:MSenseDevice` | `ECGv0` | Imported by a path-only rename in `8b27bd3359c0c24ecda10fa0c028f0c8292255df` | Full binary/file-mode comparison recorded as empty before the application merge |
| `ae58cb6:MSenseDevice` | `PPGv2` | Renamed on temporary import commit `0616e62949c718474ac2a4cd682f71942f4fcd91`, then merged with explicit ancestry in `e3e91a1a66568f26355c3877d4391af970a7a2f6` | Full binary/file-mode comparison recorded as empty before the merge commit |
| Root `.gitignore` | Root `.gitignore` | Selected from main because `adb80d2` adds only product-neutral ignores for local `datasheets/`, `nathan_ignore/`, and `.codex/` material | The ignore rules do not remove or modify existing files |
| Root `PPG_PACKED_16_BYTE_FORMAT.md` | Not imported in Phase 0 | Deferred PPG-specific format documentation; it remains recoverable through the main-side merge ancestry and must be placed deliberately with product documentation in a later phase | Source commit `91d45c8`; no root-file selection was made for it in this import-only phase |
| Root `README.md`, root DK overlay, and other unchanged root files | Existing ECG-baseline copies | Retained without modification | No differing source content at the recorded tips |

The former root `MSenseDevice` path contains no tracked source files after the
imports. Local ignored build/editor artifacts at that path were intentionally
left in place rather than moved or deleted.

## Checkpoint history

1. `8b27bd3` — `chore(ecgv0): rename cleaned ECG application without behavior changes`
2. `0616e62` — `chore(ppgv2): import main application with branch ancestry` on the temporary main import branch
3. `e3e91a1` — merge of the PPGv2 import, preserving both source ancestries
4. `0dc6b5c` — documentation-only merge of `f5a8743` from `codex/ecgv0-history-docs`

No build, flash, storage-media operation, or functional firmware change was
performed in Phase 0. Later phases must preserve the two imported application
trees until a scoped functional change is explicitly made and validated.

## Working-tree preservation

The pre-existing untracked user material was preserved. At Phase 0 start it
included `.codex/`, `FIRMWARE_CONSOLIDATION_HANDOFF.md`, `datasheets/`,
`nathan_ignore/`, and `tmp/`. After selecting main's `.gitignore`, some of
those paths are intentionally hidden from ordinary `git status`, but no file
was cleaned, removed, or moved by this phase.
