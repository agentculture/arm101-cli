# Resume here — the joint-limits programme (issue #43)

A cold-start pointer. **The authoritative write-up is the latest comment on
[#43](https://github.com/agentculture/arm101-cli/issues/43)** — read that first; this file only
tells you where to look and in what order.

## State as of 2026-07-13

| | |
|---|---|
| branch | `feat/arm-limits` |
| PR | [#48](https://github.com/agentculture/arm101-cli/pull/48) — v0.24.1, 1697 tests green |
| hardware | SO-101 follower, `/dev/ttyACM1` (a Reachy Mini is on `ttyACM0` — **always pass `--port`**) |

`arm limits` ships and works. It re-derived `elbow_flex`'s encoder offset from measurement alone
to **within one tick** (1158 vs 1157), having driven itself through the encoder seam.

## The finding that matters

**A contact threshold set above any load a joint can produce makes the joint look FREE.**
`wrist_roll` shipped with a threshold of 400 against stops that press at 172-288. Contact could
never fire, so the arm drove into two real stops and reported open air — and that reading was
written into `arm_spec` as **"PROVEN"**. A human hand finds two ends.

The guard against a repeat is **`LimitVerdict.UNFIRABLE_THRESHOLD`**
(`arm101/hardware/limits.py`): a probe that times out having never reached
`peak_load > threshold` *provably* could not have called a contact, and now says so.

**The threshold ceiling is the joint's WEAKEST wall — not 500 (that is where the *sensor*
saturates) and not the average end.** The gripper has one end at 500 and one at 284.

## Read the issues in THIS order

1. **[#51](https://github.com/agentculture/arm101-cli/issues/51)** — a COMPLIANT stop (a cable) is
   reported as a WALL, and the verdict **does not replicate**. A WALL is the only verdict that
   authorises a re-zero. **Start here.**
2. **[#52](https://github.com/agentculture/arm101-cli/issues/52)** — `arm limits` probes ONE pose;
   `shoulder_lift` and `wrist_flex` have **no honest limits at all**. Blocked on #51.
3. **[#34](https://github.com/agentculture/arm101-cli/issues/34)** — `arm explore`'s grid wants the
   measured bounds. Blocked on both.

**Do not close the circularity**: #52 wants poses, poses could come from a map, the map comes
from #34's grid, and #34 wants #52's bounds. Poses must be **hand-authored / operator-posed**.

## Gotchas that cost real time

- **Start a probe MID-TRAVEL.** A probe that begins with the joint sitting *on* an end has no free
  approach and its result is degenerate.
- **Don't reason a constant down from a real signal — measure the noise.** `_PRESSING_EXCESS_LOAD`
  was set to 25 by reasoning from a wall's ~212 excess; a false stall cleared it at **32**.
- **Trust at-rest reads, not path integration.** A 20 Hz sampler aliases a fast hand-spin: >2048
  ticks between samples and `signed_delta` picks the wrong direction.
- **`git stash` is unsafe here** — `uv run` regenerates `uv.lock` and blocks `stash pop`. Commit.
- **A latched servo (`error=32`) is cleared by `bus.clear_overload(motor)`** — which as of v0.24.0
  *verifies* the latch actually dropped. It is **not instant** (~250 ms).
