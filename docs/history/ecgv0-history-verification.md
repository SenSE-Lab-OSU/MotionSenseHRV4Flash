# ECGv0 history rewrite verification

## Source and safety references

| Item | Value |
| --- | --- |
| Refreshed main tip | `b61971b60e08a31055aec3a7ae9edd5b0b1cb7ec` |
| Original ECG tip | `070793f51ce01262724812d73e3ed47b3c2a6bd9` |
| Merge base | `68967d2d31b7ca308877b0c9e644ce5e0feb2a27` |
| Archival local ref | `refs/archive/ecgv0-original-2026-08-27` -> `070793f51ce01262724812d73e3ed47b3c2a6bd9` |
| Rewrite branch | `codex/ecgv0-history-cleanup` |
| Rewritten tip | `7aac128a5b2c2e028aebd6ea5b076680cc1cbc4b` |

The original `simp-nathan-ecg-yuyi200mAh` branch was not reset, renamed,
deleted, or force-pushed. No remote archive ref was created; that requires
separate repository-policy authorization.

## Required verification gate

| Check | Command | Result |
| --- | --- | --- |
| Source/tree count | `git rev-list --count 68967d2..070793f` / `git rev-list --count 68967d2..HEAD` | `41` source commits, `11` rewritten commits |
| Old tree | `git rev-parse 070793f^{tree}` | `fd0b7d850237693ea42918314d61fe3598a09a1b` |
| New tree | `git rev-parse 7aac128^{tree}` | `fd0b7d850237693ea42918314d61fe3598a09a1b` |
| Full tree diff | `git diff --exit-code --no-ext-diff --binary 070793f 7aac128` | Exit status `0`; no output |
| Name/status diff | `git diff --name-status --no-ext-diff 070793f 7aac128` | No output |
| Range comparison | Command recorded in `ecgv0-history-range-diff.txt` | Exit status `0`; expected squash/split/reorder structure only |

Because the old and new tree object IDs are identical and the binary,
file-mode-aware diff is empty, the rewritten range contains no main-side file
content, directory rename, generated file, formatting-only change, deletion,
or binary difference beyond the original ECG tip.

## Build status

No full NCS build was run. Repository guidance requires explicit authorization
before a full firmware build; tree identity is the non-negotiable Phase -1
gate and does not depend on a build.

## Working-tree and artifact note

Before the rewrite, the checkout contained user-owned untracked material,
including `.codex/`, `FIRMWARE_CONSOLIDATION_HANDOFF.md`, `datasheets/`,
`nathan_ignore/`, and `tmp/`. It was preserved.

The three Phase -1 review artifacts in `docs/history/` are intentionally
untracked. Committing them would make the rewritten tip’s tree differ from the
archived ECG source, violating the Phase -1 final-tree requirement. They are
ready for owner review and can be copied into a pull-request description or
placed on a separate documentation branch after approval without changing this
rewrite branch.

## Owner approval checkpoint

Do not begin Phase 0 until the owner approves this branch, the map, the
identity evidence, and the range-diff explanation.
