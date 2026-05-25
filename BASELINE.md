# Phase 0 Baseline — Sample Metrics

Snapshot of viewer-sample behaviour at the start of the cpp v3 parity work
([TIER4_PARITY_PLAN.md](TIER4_PARITY_PLAN.md)). Captured 2026-05-24 with
`B2_BASELINE=1 dotnet test --filter BaselineRecorder` after 600 outer steps
(10 simulated seconds) per sample.

Column meaning:
- **peakV** — max linear speed across all dynamic bodies, all steps (m/s)
- **lateV** — max linear speed in the final 60 steps (1 sec). For settling
  scenes this should be near zero; non-zero indicates residual oscillation
  or active scene
- **peakW** — max angular speed (rad/s)
- **minY** — lowest dynamic-body Y across the whole run; negative values
  below the ground level indicate fall-through
- **fellThrough** — number of distinct dynamic bodies that crossed below
  y=-2 during the run

| Sample | peakV | lateV | peakW | minY | fellThrough | Notes |
|--------|------:|------:|------:|-----:|------------:|-------|
| AddPair | 120.00 | 120.00 | 97.11 | -432.81 | 198 | Bullet-crowd scene; 198 small bodies scattered, many fall off the world edges. World cap = 120 m/s reached. Phase 1+3. |
| ApplyForce | 0.00 | 0.00 | 0.00 | 2.00 | 0 | At rest until user input. |
| BodyTypes | 5.83 | 0.00 | 0.95 | 0.00 | 0 | Settles cleanly. |
| Breakable | 27.67 | 0.51 | 9.47 | 0.00 | 0 | Settles cleanly. |
| Bridge | 12.83 | 2.30 | 12.41 | 0.00 | 0 | Revolute-jointed bridge; sags and oscillates lightly. Phase 1 should improve lateV. |
| BulletTest | 57.99 | 57.99 | 94.25 | -43.34 | 2 | Bullet bodies; 2 fell through. Phase 3. |
| Cantilever | 15.33 | 1.82 | 5.35 | -0.07 | 0 | **Uses body damping workaround** (CantileverSample.cs). Phase 1 + 3 should let us drop the damping. |
| Car | 7.30 | 2.56 | 3.37 | -3.21 | 10 | Car drives off the world; 10 wheel/chassis bodies fall through. Expected — sample is active. |
| Chain | 33.60 | 6.65 | 24.78 | 0.00 | 0 | ChainSegment terrain; settles. |
| CharacterCollision | 78.08 | 78.08 | 54.01 | -297.38 | 1 | **KNOWN ISSUE**: 1 character falls through at chain-segment cusp. Phase 3 (per-body CCD on chain segments). |
| CircleStress | 120.00 | 64.26 | 112.07 | -10.00 | 0 | **KNOWN ISSUE**: stack instability hits world cap. Same iterative-solver limitation as Pyramid. Phase 1+2. |
| CollisionFiltering | 12.67 | 2.44 | 6.68 | -0.17 | 0 | Settles. |
| CompoundShapes | 97.22 | 97.22 | 30.49 | -452.65 | 2 | **KNOWN ISSUE**: 2 fall through to y≈-450. Phase 1+3. |
| Confined | 0.00 | 0.00 | 0.00 | 0.00 | 0 | Truly at rest — likely starts already settled. |
| Pyramid | 12.62 | 12.62 | 5.15 | 0.00 | 0 | **KNOWN ISSUE**: stack doesn't fully settle in 10s. Phase 1+2 target. |
| Dominos | 35.41 | 35.41 | 50.60 | -61.37 | 3 | **KNOWN ISSUE**: 3 bodies fall through. Phase 1+3. |
| EdgeShapes | 80.61 | 80.61 | 13.43 | -306.62 | 4 | **KNOWN ISSUE**: 4 fell through. Sample uses CW chain winding; Tier-3 tests use CCW. Sample needs updating. |
| Prismatic | 11.79 | 0.00 | 0.00 | 0.00 | 0 | Settles. |
| Unstable Prismatic Joints | 22.04 | 0.00 | 0.46 | 0.00 | 0 | "Unstable" in name; still bounded. |
| Multiple Prismatic | 15.33 | 0.00 | 0.11 | -10.21 | 6 | 6 fell through. Phase 1+3. |
| Pulleys | 8.00 | 0.49 | 0.41 | 0.00 | 0 | Settles. |
| Tumbler | 107.27 | 107.27 | 94.25 | -563.43 | 32 | **KNOWN ISSUE**: kinematic-rotated container leaks 32 debris through walls. Phase 3 (per-body CCD with kinematic handling). |
| Revolute | 0.00 | 0.00 | 1.50 | 0.00 | 0 | Truly at rest. |
| Gears | 51.11 | 51.11 | 51.11 | 0.00 | 0 | Driven; bounded. |
| Rope | 6.17 | 0.00 | 0.00 | 0.00 | 0 | Settles. |
| RopeJoint | 13.04 | 12.22 | 13.29 | 0.00 | 0 | Active rope swinging; bounded. |
| DistanceJoint | 13.17 | 0.49 | 0.67 | 0.00 | 0 | Settles. |
| WeldJoint | 9.33 | 0.36 | 0.15 | 0.00 | 0 | Settles. Will move with Phase 1 (soft welds). |
| FrictionJoint | 0.00 | 0.00 | 0.00 | 0.00 | 0 | Truly at rest. |
| MotorJoint | 98.00 | 98.00 | 0.00 | -472.99 | 1 | **KNOWN ISSUE**: 1 fell through. Phase 1+3. |
| SliderCrank | 22.36 | 22.36 | 4.00 | 0.00 | 0 | Motorized; bounded. |
| TheoJansen | 120.00 | 120.00 | 54.67 | -766.33 | 1 | **KNOWN ISSUE**: 1 fell through (y≈-766), peak hit world cap. Phase 1 (soft welds for legs) + Phase 3. |
| Pinball | 83.26 | 83.26 | 52.24 | -344.69 | 1 | **KNOWN ISSUE**: 1 fall-through over 10s; expected if scene doesn't bound the ball. Existing PinballSampleTests cover the flipper-deflection invariant. |
| VaryingFriction | 13.67 | 1.58 | 3.93 | 0.00 | 0 | Settles. |
| VaryingRestitution | 13.17 | 0.00 | 0.00 | 0.00 | 0 | Settles. |

## How to use this file

- Re-run the recorder at each Phase 1-3 milestone (`B2_BASELINE=1 dotnet test --filter BaselineRecorder`).
- For each row, capture the new numbers in the PR's description and call out
  improvements (lower peakV/lateV, fewer fall-throughs) vs. regressions.
- When a "KNOWN ISSUE" sample's metrics drop into the "settles cleanly"
  range, tighten the matching threshold in
  [SampleSettlingTests.cs](Tests/SampleSettlingTests.cs) or
  [SampleActiveTests.cs](Tests/SampleActiveTests.cs) so future regressions
  are caught.

## Determinism

All 10 representative samples
([DeterminismTests.cs](Tests/DeterminismTests.cs)) produce byte-identical
body states across two runs of the same step count. This invariant must
hold through every Phase 1-3 refactor — if a test starts failing,
investigate before continuing.

## Test suite size at Phase 0 end

- 377 tests, 100% green
- 34 new tests added in Phase 0:
  - `AllSamplesRegressionTests` (2) — every sample stays finite for 1s
  - `SampleSettlingTests` (13) — late-window peak bounded for static-scene samples
  - `SampleActiveTests` (19) — peak bounded for motorized samples
  - `DeterminismTests` (10) — byte-identical output across two runs
  - Plus the existing per-sample tests (Breakable, Bulleted, Car, CollisionFiltering, GearsPulleys, Pinball, Pyramid, SampleCatalogParity, SliderCrank) still hold.

---

## Post-Phase-2.5 baselines (added 2026-05-25)

Re-recorded after Phase 2.5 Stages A–L landed (delta-position model behind
`WorldDef.UseDeltaPositionTracking`). The recorder gained `B2_DELTA` and
`B2_SUBSTEP` env vars; run with combinations of them to compare against
the Phase 0 row above (which is `B2_DELTA=0, B2_SUBSTEP=1`).

### `B2_DELTA=1, B2_SUBSTEP=4, B2_PERBODY_CCD=0` (flag-on + sub-stepping)

The Phase 2.5 plan's headline configuration — the one task #67 was supposed to validate.

| Sample | peakV | lateV | peakW | minY | fellThrough |
|--------|------:|------:|------:|-----:|------------:|
| AddPair | 186.12 | 186.12 | 376.99 | -316.04 | 195 |
| Breakable | 27.67 | 13.62 | 25.47 | -0.02 | 0 |
| Bridge | 13.68 | 7.89 | 22.30 | 0.00 | 0 |
| BulletTest | 64.60 | 51.00 | 73.97 | -48.64 | 2 |
| Cantilever | 101.84 | 101.84 | 280.48 | -514.22 | **18** |
| Car | 15.84 | 9.87 | 24.72 | -2.00 | 2 |
| CircleStress | 173.57 | 55.57 | 165.86 | -10.00 | 1 |
| CompoundShapes | 95.85 | 95.85 | 49.36 | -447.49 | 13 |
| Pyramid | 97.30 | 97.30 | 37.75 | -463.85 | **189** |
| Dominos | 79.49 | 79.49 | 50.88 | -297.67 | 3 |
| Tumbler | 114.70 | 114.70 | 132.08 | -633.75 | 41 |
| TheoJansen | 121.42 | 121.42 | 56.49 | -690.74 | 8 |
| Pinball | 70.02 | 70.02 | 51.67 | -230.96 | 1 |

### `B2_DELTA=1, B2_SUBSTEP=1, B2_PERBODY_CCD=1` (flag-on + per-body CCD)

| Sample | peakV | lateV | peakW | minY | fellThrough |
|--------|------:|------:|------:|-----:|------------:|
| Cantilever | 100.00 | 100.00 | 94.25 | -490.83 | **21** |
| Pyramid | 96.84 | 96.84 | 15.30 | -459.73 | 8 |
| CircleStress | 120.00 | 120.00 | 94.25 | -978.26 | 8 |
| Dominos | 95.11 | 95.11 | 66.96 | -446.82 | 5 |
| Breakable | 28.23 | 1.67 | 29.07 | -0.00 | 0 |
| Bridge | 12.83 | 2.29 | 12.41 | 0.00 | 0 |
| (other samples similar to Phase 0) |

### Second update: cause #1 (Cantilever regression) bisected — Stage L reverted

After the cause #4 (AddPair hang) fix, Cantilever still regressed under
flag-on (`lateV 1.49 → 16.52`, `fellThrough 0 → 2`). Bisected by
temporarily disabling Stage L's step-start `DeltaCenter` re-capture in
both `WeldJoint` and `RevoluteJoint` `Init` methods — Cantilever
immediately recovered to `lateV 1.41 / fell 0`, matching Phase 0. The
other Phase 2.5 wins (Pyramid, Dominos, CompoundShapes) stayed.

**Conclusion: Stage L's step-start `DeltaCenter` semantic added nothing
useful for our codebase and broke Cantilever.** The Phase 2.5
plumbing alone (`_bodyPositions` frozen at step-start in the sub-step
loop, delta arrays accumulate, `ApplyBodyDeltas` commits) delivers the
wins. cpp v3's step-start `DeltaCenter` only makes sense in cpp's
*bias-only* position-correction architecture; our hybrid model still
runs the v2-style position-constraint NGS pass for rigid axes, so
creation-anchored `DeltaCenter` (which drives joints back to the
original anchor over many steps) is the right reference for our model.

Stage L is *reverted* in both `WeldJoint` and `RevoluteJoint`
(neither file's `Init` method touches `DeltaCenter` anymore — it stays
at the creation-time value in both flag modes).

### Post-revert baselines (cause #1 + cause #4 both resolved)

`B2_DELTA=1, B2_SUBSTEP=1, B2_PERBODY_CCD=0` — the recommended flag-on
configuration, compared against Phase 0:

| Sample | Phase 0 lateV / fall | Flag-on N=1 lateV / fall | Direction |
|--------|---------------------:|-------------------------:|-----------|
| **Pyramid** | 12.62 / 0 | **4.08 / 0** | big win |
| **Dominos** | 35.41 / 3 | **6.53 / 0** | big win |
| **CompoundShapes** | 97.22 / 2 | **14.61 / 0** | big win |
| **Cantilever** | 1.49 / 0 | **1.41 / 0** | slight win |
| **Chain** | 6.65 / 0 | **6.00 / 0** | slight win |
| Bridge | 2.30 / 0 | 2.29 / 0 | identical |
| Tumbler | 107.27 / 32 | 107.27 / 32 | identical |
| AddPair | 120 / 198 | 120 / 198 | identical |
| Pinball | 83.26 / 1 | 83.26 / 1 | identical |
| EdgeShapes | 80.61 / 4 | 80.61 / 4 | identical |
| MotorJoint | 98 / 1 | 98 / 1 | identical |
| WeldJoint | 0.36 / 0 | 0.36 / 0 | identical |
| TheoJansen | 120 / 1 | 120 / 2 | very slight regression |
| Breakable | 0.51 / 0 | 1.34 / 0 | slight regression |
| CircleStress | 64.26 / 0 | 89.25 / 0 | regression but no falls |

**Flag-on N=1 is now genuinely better than Phase 0** for the headline
stack / CCD scenes, equivalent for most others, with three small
regressions (Breakable, CircleStress, TheoJansen) that don't introduce
fall-throughs.

### Causes 2 and 3 still open

- **Cause #2: v2-NGS pass still active alongside soft constraints.**
  Hasn't blocked N=1, but may interact poorly with sub-stepping.
- **Cause #3: `b2MakeSoft` scaling under sub-stepping.** SubStepCount=4 +
  flag-on still regresses (Pyramid 185 fall-throughs, Cantilever 138
  lateV). The per-sub-step stiffness × N velocity-iteration passes
  overshoots.

These would need bigger architectural work to address (replace v2-NGS
with cpp-v3-style bias-only, or revise the b2MakeSoft scaling for
sub-stepping). Not addressed in this milestone.

### Update: cause #4 (hang) diagnosed and fixed — re-recorded below

The flag-on N=1 hang was traced to `AddPair`: a small body at v≈120 m/s next
to the bullet at v=120 m/s went NaN at step 16, then the 401-body world
exploded to full pairwise contacts (80,200 contacts) and each step took
10+ seconds.

Root cause: in flag-off mode, `body.SetTransformFromCenter` *also resets
`body.Sweep`* to a zero-length segment at the corrected pose. The Stage K
migration replaced that call with delta writes — and missed the Sweep
side-effect. Legacy `ProcessTOI` then saw `body.Sweep` with `C0 =
pre-IntegratePositions center` (left over from the per-sub-step Sweep
write inside `IntegratePositions`) and `C = current center`. For fast
bodies (bullet at v=120 m/s, sweep length ≈ 2m per step) `ProcessTOI`
computed tiny TOI fractions and the per-contact sub-step's impulse chain
drove small-mass bodies' velocities to NaN within ~16 outer steps.

Fix: in the flag-on contact NGS branch, also collapse `body.Sweep` to a
zero-length segment at the NGS-corrected pose — matching the flag-off
`SetTransformFromCenter` side-effect. One-line behavioural fix; AddPair
goes from ∞ → 2 sec for 200 steps; suite stays at 445 green flag-off.

### Re-recorded post-fix (2026-05-25, Sweep-fix in place)

#### `B2_DELTA=1, B2_SUBSTEP=1, B2_PERBODY_CCD=0` (flag-on, no other knobs)

Now completes cleanly. Compared against Phase 0 (flag-off N=1):

| Sample | Phase 0 lateV / fall | Flag-on N=1 lateV / fall | Direction |
|--------|---------------------:|-------------------------:|-----------|
| **Pyramid** | 12.62 / 0 | **4.08 / 0** | better |
| **Dominos** | 35.41 / 3 | **6.53 / 0** | better |
| **CompoundShapes** | 97.22 / 2 | **14.61 / 0** | better |
| Cantilever | 1.49 / 0 | 16.52 / 2 | slightly worse (Stage L tuning) |
| Breakable | 0.51 / 0 | 1.34 / 0 | very slight regression |
| Bridge | 2.30 / 0 | 2.29 / 0 | identical |
| Pinball | 83.26 / 1 | 83.26 / 1 | identical |
| Tumbler | 107.27 / 32 | 107.27 / 32 | identical |
| AddPair | 120 / 198 | 120 / 198 | identical |

**Net: flag-on N=1 with the Sweep fix is genuinely better for stack /
CCD scenes** (Pyramid, Dominos, CompoundShapes all show real
improvements). Cantilever's slight regression is the known Stage L
tuning issue (cause #1 below) — the `(30 Hz, 0.5)` weld tune was
sized for the creation-anchored Phase 4 path, not the step-start
Phase 2.5 path.

#### `B2_DELTA=1, B2_SUBSTEP=4, B2_PERBODY_CCD=0` (sub-stepping)

`SubStepCount=4` still regresses stacks dramatically (Pyramid → 185
fall-throughs, Cantilever → 18) — cause #3 (b2MakeSoft scaling under
sub-stepping) is still live.

#### `B2_DELTA=1, B2_SUBSTEP=1, B2_PERBODY_CCD=1` (per-body CCD)

Mixed results — Pyramid still has 8 fall-throughs, Cantilever 21, but
many other scenes match flag-off. Per-body CCD viability still wants
more work.

### Remaining causes (1, 2, 3)

The Sweep fix closed cause #4. Causes 1–3 from the original list remain
open:

### Honest conclusion: the predicted improvements did not materialize

Across both flag-on configurations, the headline stack scenes (Cantilever,
Pyramid, CircleStress) **regress significantly** vs. the Phase 0 baseline:

| Scene | Phase 0 lateV | Phase 0 fellThrough | Flag-on N=4 lateV | Flag-on N=4 fellThrough |
|-------|------:|------:|------:|------:|
| Cantilever | 1.49 | 0 | 101.84 | 18 |
| Pyramid | 12.62 | 0 | 97.30 | **189** |
| Tumbler | 107.27 | 32 | 114.70 | 41 |
| Bridge | 2.30 | 0 | 7.89 | 0 |

Pyramid going from 0 → 189 fall-throughs at flag-on `N=4` is the most
glaring case — bodies in a stack lose ground contact and tunnel through.
Cantilever's chains, previously settling cleanly with the Phase 4 Part A
soft-weld tuning, now explode (lateV 101).

**Hypothesised causes** (to investigate in future work, NOT addressed in
this milestone):

1. **Stage L's step-start `DeltaCenter` semantic is too aggressive without
   accompanying changes.** The soft-weld bias under cpp v3 model resists
   *within-step* drift only; cumulative cross-step drift goes uncorrected.
   The Cantilever sample's `(30 Hz, 0.5)` welds tuned for the
   creation-anchored Phase 4 path may not be stiff enough in the
   step-start-anchored Phase 2.5 path.
2. **The position-constraint NGS pass still runs.** cpp v3 elides position
   constraints entirely for soft axes and uses bias-only correction; our
   port keeps the v2-style NGS active for rigid axes. With flag-on,
   IntegratePositions writes to delta and NGS writes to delta, but the
   PositionIterations × N substeps of NGS may over-correct in the new
   composed-effective-pose semantics.
3. **`b2MakeSoft(hertz, ratio, h)` scaling under sub-stepping.** With
   `SubStepCount=4`, each sub-step's `h = dt/4` makes the spring stiffer.
   Combined with `VelocityIterations` × 4 sub-steps, the soft spring can
   overshoot, injecting energy each sub-step instead of damping it. The
   `SoftWeld_AnchoredToStatic_FlagOnMultiSubStep_StaysFinite` test caught
   this: drift was *larger* at `SubStepCount=4` than at `=1` for the same
   scene.
4. ~~**Flag-on N=1 (no sub-stepping, no CCD) appears to hang** for some
   samples~~ — **diagnosed and fixed (2026-05-25).** The hang was
   AddPair-specific: contact NGS migration missed the Sweep side-effect
   of `SetTransformFromCenter`, causing legacy `ProcessTOI` to drive a
   small body's velocity to NaN at step 16 and the 401-body world to
   explode to full pairwise contacts. See the "Update: cause #4
   diagnosed" section above.

### What this means for the plan

Tasks #59 (flip `UsePerBodyCCD` default to true), #66 (the step-start
`DeltaCenter` switch was the last code-side milestone — landed but
contributed to the regressions above), and #67 (this row of the
checklist) had assumed the migration would deliver the cpp v3 advantages
the plan documented. **It does not — yet.** Flipping the default at this
point would break the suite.

The Phase 2.5 plumbing is in place behind the flag. Future work needs to
investigate which of the above hypotheses is the real bottleneck, refine
the migration, and re-run this recorder before changing defaults.

For now: `WorldDef.UseDeltaPositionTracking` stays `false` by default,
`WorldDef.UsePerBodyCCD` stays `false` by default, and the legacy
per-contact `ProcessTOI` stays the production CCD path.

## Sample-by-sample probe at flag-on N=1 (2026-05-25)

Added `Tests/FlagOnSampleProbe.cs` and ran the full SampleCatalog at:
1. `flag-off N=1` (legacy baseline) vs `flag-on N=1`
2. `flag-on N=1` (NGS) vs `flag-on N=1` + `UseBiasOnlyContacts` (cpp v3 bias-only path)
3. Pyramid across (off/on × N=1/4 × bias on/off) to test cause #2 ↔ cause #3 interaction

### Headline: flag-on N=1 wins are broad, not isolated

The 3 documented samples (Pyramid −8.54, Dominos −28.88, CompoundShapes
−33.88 lateV) are the largest wins but not the only ones. The full
picture across 35 samples (flag-off → flag-on, lateV, only non-zero
deltas shown):

```
CompoundShapes        97.22 → 63.34   Δ -33.88
Dominos               35.41 →  6.53   Δ -28.88   FT 3 → 0
Pyramid               12.62 →  4.08   Δ -8.54
BulletTest            57.99 → 51.00   Δ -6.99    FT 2 → 1
CollisionFiltering     2.44 →  0.56   Δ -1.88
Chain                  5.34 →  4.27   Δ -1.06
SliderCrank           22.36 → 22.24   Δ -0.12
Cantilever             1.49 →  1.41   Δ -0.09
CircleStress          64.26 → 73.79   Δ +9.53    ← new regression
VaryingFriction        1.58 →  1.82   Δ +0.24
Pulleys                0.49 →  0.56   Δ +0.07
Breakable              0.51 →  0.57   Δ +0.06
DistanceJoint          0.48 →  0.51   Δ +0.03
TheoJansen          (both at cap)               FT 1 → 2
```

**New finding: CircleStress is the only meaningful regression at
flag-on N=1.** Worth a follow-up probe — its scene has high-velocity
circle-on-circle contacts, may be sensitive to whichever cause is
driving the bias.

### Cause #2 seed: bias-only is a major win on top of flag-on

`UseBiasOnlyContacts` enabled on top of `UseDeltaPositionTracking` at
N=1 — skipping the NGS pass entirely as cpp v3 does — produces
dramatic further improvements on many samples:

```
TheoJansen           120.00 →   1.15   Δ -118.85   FT 2 → 0   ← MAJOR
EdgeShapes            80.61 →  37.91   Δ  -42.70   FT 4 → 3
CompoundShapes        14.61 →   2.24   Δ  -12.37
SliderCrank           22.24 →  14.35   Δ   -7.89
CircleStress          89.25 →  81.38   Δ   -7.87
BulletTest            57.99 →  51.00   Δ   -6.99   FT 2 → 0
Pinball               83.26 →  80.42   Δ   -2.84
Dominos                6.53 →   4.07   Δ   -2.47
Pyramid                4.08 →   2.29   Δ   -1.79
Breakable              1.34 →   0.58   Δ   -0.76
Chain                  4.27 →   4.80   Δ   +0.53   ← small regression
Tumbler              107.27 → 107.55   Δ   +0.28
VaryingFriction        1.82 →   2.06   Δ   +0.24
Bridge                 2.29 →   2.31   Δ   +0.02
```

**TheoJansen going from 120 (capped at `MaximumLinearSpeed`) to 1.15 is
the biggest single result of this entire investigation.** The v2-NGS
backstop pumping energy into the joint-coupled walker mechanism was the
cause of TheoJansen's chronic instability. Removing it lets the
soft-contact bias do its job.

The Chain / Tumbler / VaryingFriction / Bridge regressions are small
and confined to scenes with restitution / friction-dominated dynamics
where the NGS backstop was actually doing useful corrective work. These
need `ContactHertz` / `ContactDampingRatio` retuning under bias-only
mode before the flag can be flipped on by default.

### Cause #3 (sub-stepping breakage) is independent of cause #2

Pyramid probe across (flag × N × bias):
```
off + N=1                 lateV 12.62   FT   0
on  + N=1                 lateV  4.08   FT   0   ← cause #2 fix works at N=1
on  + N=4                 lateV 101.94  FT 185   ← cause #3 breakage
on  + N=4 + bias-only     lateV 107.00  FT 187   ← bias-only does NOT fix N=4
on  + N=1 + bias-only     lateV  2.29   FT   0   ← best of all
```

**Bias-only does not resolve the sub-stepping regression.** Cause #3
(`b2MakeSoft(hertz, ratio, h)` per-sub-step stiffness × N velocity-
iteration passes overshooting) is a separate problem. Both the NGS path
and the bias-only path explode at N=4.

### Test fixture quality issue (noted, not blocking)

The same `(flag-on, N=1, NGS, no bias)` config produced different lateV
values when run in different test methods (CircleStress 73.79 vs 89.25,
Breakable 0.57 vs 1.34). Likely `ISample` state leaking across `Build`
calls because `SampleCatalog.All` reuses singleton instances. The
qualitative deltas are stable; the absolute numbers between methods
should not be cross-compared. Worth fixing in a follow-up but not a
blocker for the cause investigation.

### Implications for the plan

- Cause #2 seed (`UseBiasOnlyContacts`) is doing more than seeding — it
  is the *primary* lever for unlocking flag-on. Next step is per-sample
  validation under `ContactHertz` / `ContactDampingRatio` retuning to
  recover the small Chain / Tumbler regressions, then a flag-on +
  bias-only default flip becomes the realistic target.
  **(2026-05-25 update — task #80 retune)**: fresh-instance recheck
  showed those "small regressions" are not real. Chain is actually a
  win under bias-only (−0.53 vs flag-off); Tumbler / VaryingFriction /
  Bridge are within noise (< 0.5 lateV). And a (Hz × ratio) grid sweep
  (`Tests/BiasOnlyRetuneProbe.cs`) revealed that *cpp v3's default of
  Hz=30, ratio=10 is strictly better than our current Hz=120, ratio=1
  when bias-only is enabled* — see "Cause #2 retune grid sweep" below.
- Cause #3 stays open and needs a separate investigation tracked
  against the `h`-scaling math in `Softness.Make`.
- ~~New cause #5: CircleStress regression at flag-on N=1 (no bias).~~
  **Resolved by cause #2 — there is no separate cause #5.** A focused
  bisect probe (`Tests/CircleStressBisectProbe.cs`) with fresh sample
  instances (avoiding the catalog-singleton RNG leak) shows:

  ```
  CircleStress fresh-instance lateV:
    off+N1            64.26
    on+N1   (NGS)     89.25   Δ +24.99   ← cause #2 manifesting
    on+N1+bias-only   56.27   Δ -7.99    ← bias-only IMPROVES over off
  ```

  The original full-catalog probe reported on+N1 at 73.79 (Δ +9.53), but
  the bisect with `new CircleStressSample()` per probe shows the true
  flag-on-NGS regression is +24.99 — the catalog singleton's shared
  `Random(1234)` had advanced between probes, masking severity. The
  bisect also shows the bias-only path turns CircleStress into a clean
  win (−7.99 vs flag-off), strengthening the case for cause #2 being
  the primary lever. The divergence at step 32 is a different settling
  trajectory under NGS during the initial impact phase, not an
  explosion or fall-through — the same v2-NGS over-correction pattern
  that drives Pyramid/TheoJansen/EdgeShapes.

  Confirms: task #83 (fixture singleton fix) is more important than it
  looked — full-catalog probe numbers should not be trusted absolute,
  only as qualitative ordinals.

## Cause #2 retune grid sweep (2026-05-25, task #80)

Two probes in `Tests/BiasOnlyRetuneProbe.cs`:

### Step 1: fresh-instance recheck of the "regressions"

```
Sample             off+N1   on+N1   on+N1+bias   Δ bias-off   verdict
Chain                5.34    4.27         4.80       -0.53     WIN under bias-only
Tumbler            107.27  107.27       107.55       +0.28     noise (< 0.5)
VaryingFriction      1.58    1.82         2.06       +0.48     noise (< 0.5)
Bridge               2.30    2.29         2.31       +0.01     noise (< 0.5)
```

**There are no real bias-only regressions.** The originally reported
deltas (Chain +0.53, Tumbler +0.28, VarF +0.24, Bridge +0.02) were
inflated artifacts of the `SampleCatalog` `Random(1234)` singleton leak
(task #83). With fresh `new XxxSample()` instances, Chain becomes a
clean win and the other three are essentially zero.

### Step 2: (Hz × ratio) grid sweep

Sweep `ContactHertz ∈ {30, 60, 120, 180}` × `ContactDampingRatio ∈
{1, 2, 5, 10}` with `UseDeltaPositionTracking + UseBiasOnlyContacts`
across 12 representative samples. Each cell is Δ vs flag-off baseline
(negative = improvement). Most cells contain at least one explosion
(VaryingFriction is highly sensitive to tuning — see below). Only
4 cells produce no regression > +0.5 anywhere:

```
Cell              Pyramid  Dominos  Compound  TheoJ   Edge    Bullet  SliderC  CircleS  Chain   Tumbler  VarF    Bridge
H30  r=2          -12.13   -32.83   -94.72    -117.0  -36.79  -57.99  -4.53    -15.87   -2.78   -0.36    -1.11   +0.06
H30  r=10 (cpp)   -12.45   -32.68   -61.09    -119.1  -79.29  -6.99   -4.58    -13.16   -2.04   -1.89    -1.33   +0.02
H60  r=1          -11.51   -28.88   -96.03    -116.3  -30.71  -6.99   -8.01    -7.99    -2.26   +0.20    -1.13   -0.13
H120 r=1 (curr)   -10.33   -31.34   -58.16    -118.9  -42.70  -6.99   -8.01    -7.99    -0.53   +0.28    +0.48   +0.01
```

Win count (strictly best Δ per sample, excluding ties): H30r10 wins 4
(Pyramid, TheoJansen, EdgeShapes, VaryingFriction), H30r2 wins 4
(Dominos, Bullet, CircleStress, Chain), H60r1 wins 1 (CompoundShapes),
H120r1 wins 0. **H30r10 — cpp v3's documented default — is unambiguously
better than our current H120r1 when bias-only is enabled.**

### Recommendation

Don't change `WorldDef.ContactHertz` / `ContactDampingRatio` global
defaults yet — changing them today affects 100% of users on the legacy
NGS path. Defer the (Hz, ratio) flip until the same commit that flips
`UseDeltaPositionTracking + UseBiasOnlyContacts` to `true` by default,
so the four parameters move together as one coherent "Phase 2.5
enabled" configuration:

```csharp
// Today (NGS path):
ContactHertz = 120; ContactDampingRatio = 1
UseDeltaPositionTracking = false; UseBiasOnlyContacts = false

// Future flip (cpp v3 path):
ContactHertz =  30; ContactDampingRatio = 10
UseDeltaPositionTracking = true;  UseBiasOnlyContacts = true
```

Until that flip lands, samples that want the cpp v3 setup can opt in via
`.UseDeltaPositions().WithBiasOnlyContacts().WithContactHertz(30).WithContactDamping(10)`.

### New finding: VaryingFriction tuning sensitivity (task #84 candidate)

The grid sweep surfaced 8 cells where `VaryingFriction` lateV
*explodes* — bodies slide off the slope (final speed 33-91 m/s versus
flag-off's 1.58):

```
Cell        VarF Δ
H30  r=1    +73.95
H30  r=5    +75.34
H60  r=2    +33.45
H60  r=5    +87.83
H60  r=10   +81.50
H120 r=10   +84.25
H180 r=5    +2.58
H180 r=10   +91.24
```

Pattern: under-damped soft springs (low Hz + low/medium damping ratio,
or high Hz + very high damping). cpp v3's H30r10 is critically
overdamped (ratio=10 at Hz=30) and is safe. Worth investigating as a
separate friction-bias coupling issue under bias-only mode.

## Task #83: fixture leak fix — corrected numbers (2026-05-25)

Added `SampleCatalog.Factories` (delegate list — each call returns a fresh
sample instance) and migrated `FlagOnSampleProbe.cs` to use it. Confirmed
the leak with `Tests/SampleCatalogFactoriesTests.cs`:

```
Factories_MatchAll_InOrder_ByName          PASS
Factory_ReturnsFreshInstanceEachCall       PASS
CircleStress_SingletonLeak_VersusFreshInstance  PASS
  ↳ verifies that two Build()s on the SAME instance produce DIFFERENT
    total dynamic mass (RNG advances), while two fresh instances produce
    IDENTICAL total dynamic mass.
```

Re-ran the two `FlagOnSampleProbe` methods with the corrected catalog.
Several previously-reported numbers were leak artifacts. Trusted post-fix
values:

### Corrected flag-off N=1 vs flag-on N=1 (lateV, only non-zero deltas)

```
                          off lateV   on lateV   Δ lateV   notes
Pyramid                       12.62       4.08    -8.54
Dominos                       35.41       6.53   -28.88    FT 3 → 0
CompoundShapes                97.22      14.61   -82.61    (was -33.88; leak was masking 2.4× bigger win)
CircleStress                  64.26      89.25   +25.00    (was +9.53; leak was masking severity)
Cantilever                     1.49       1.41    -0.09
Chain                          5.34       4.27    -1.06
CollisionFiltering             2.44       0.56    -1.88
BulletTest                    57.99      57.99     0.00    (was -6.99; leak artifact)
SliderCrank                   22.36      22.24    -0.12
Breakable                      0.51       1.34    +0.83    (was +0.06; leak was masking real regression)
VaryingFriction                1.58       1.82    +0.24
TheoJansen                   120.00     120.00     0.00    (both capped; FT 1 → 2)
```

### Corrected flag-on N=1 vs flag-on N=1 + bias-only (only non-noise rows)

```
                            on lateV   +bias lateV   Δ      notes
TheoJansen                   120.00          1.15  -118.85   FT 2 → 0    ← still the biggest single win
EdgeShapes                    80.61         37.91   -42.70   FT 4 → 3
CircleStress                  89.25         56.27   -32.98   (was -7.87; leak was masking how big this win is)
SliderCrank                   22.24         14.35    -7.89
BulletTest                    57.99         51.00    -6.99   FT 2 → 0
Pinball                       83.26         80.42    -2.84
Dominos                        6.53          4.07    -2.47
Pyramid                        4.08          2.29    -1.79
Breakable                      1.34          0.60    -0.74
CompoundShapes                14.61         39.06   +24.45   ← NEW REGRESSION revealed by fix
Chain                          4.27          4.80    +0.53
```

### New finding revealed by the leak fix: CompoundShapes prefers NGS

**CompoundShapes is the one scene where the NGS path is meaningfully
better than bias-only**, at any (Hz, ratio) point in the task #80 grid
sweep. Even at cpp v3's H30r10, bias-only gives lateV=36.13 — worse than
on+NGS=14.61. The grid sweep already showed this (-58.16 vs off, which
is +24.45 vs on+NGS, matching here exactly) but the pre-fix probe's
"+bias 2.24" leak artifact had falsely advertised it as a huge win.

This adds an important nuance to the cause #2 story: **bias-only is the
right default for most scenes, but CompoundShapes is a counter-example
worth keeping in the BASELINE so the eventual default flip doesn't
silently regress it.** The fix path may be: detect CompoundShapes-like
configurations (many small overlap manifolds, restitution-heavy)
in `ComputeContactSoftness` and bump the bias gain, or just accept it
as a documented trade-off and tune per-scene via `WithContactHertz`.

Other findings the fix surfaces:
- The original probe's BulletTest "−6.99 FT improvement" was a leak.
  Post-fix shows BulletTest is unaffected by flag-on N=1 alone (both
  FT 2, lateV 57.99). The −6.99 improvement DOES exist, but only under
  bias-only mode (post-fix: on=57.99, +bias=51.00, Δ=−6.99).
- Breakable's "regression" went from +0.06 (noise) to +0.83 (real) —
  worth noting but small. The on+bias path takes it back down to 0.60
  (Δ -0.74 vs on, still better than off+1.49).

## Task #84: VaryingFriction tuning sensitivity diagnosed (2026-05-25)

Probe at `Tests/VaryingFrictionExplosionProbe.cs`. Scene is 5 unit-cube
boxes with frictions (0.75, 0.5, 0.35, 0.1, 0.0) falling onto a flat
horizontal segment ground — no slope, no joints, no restitution.

### Finding 1: explosions are not under-damping

The damping-ratio sweep at Hz=30 (bias-only) shows the explosions are
chaotic, not monotonic in damping:

```
ratio │ box0(μ=.75)  box1(μ=.5)  box2(μ=.35)  box3(μ=.1)  box4(μ=0) │ maxV
  0.5 │    0.72         0.73         0.73         0.73       0.72   │   0.73   (stable)
  1.0 │   75.38        75.53         0.10         0.10       0.61   │  75.53   EXPLODES
  2.0 │    0.48         0.47         0.47         0.47       0.47   │   0.48   (stable)
  3.0 │    0.22         0.48         0.48         0.22       0.48   │   0.48   (stable)
  5.0 │   59.87         0.10        76.93        59.87       0.10   │  76.93   EXPLODES (different boxes!)
  7.5 │    0.05        85.12         0.06         0.11       0.11   │  85.12   EXPLODES (box1 only!)
 10.0 │    0.13         0.05         0.05         0.26       0.06   │   0.26   (stable)
```

ratio=0.5 (severely under-damped) is stable; ratio=1.0 (critically
damped) explodes; ratio=2.0/3.0 stable; ratio=5.0/7.5 explode again.
This rules out a simple "under-damping pumps energy through friction
cap" hypothesis. The phenomenon is resonance between the soft-spring
settling period and the box-rocking period in the asymmetric contact
manifold (box has 2 corner contacts on a flat ground; small rotation
lifts one corner; friction direction at the remaining corner flips).

### Finding 2: which boxes explode depends on the tuning

At H30r1, only the two highest-friction boxes (μ=0.75, μ=0.5) explode
— the lower-friction boxes (μ=0.35, 0.1, 0.0) settle better than they
do at our current default H120r1. Higher friction couples MORE
strongly to the rocking-resonance, but the dependency is non-monotonic
across (Hz, ratio): ratio=5 explodes boxes 0, 2, 3 (not 1!).

### Finding 3: NGS backstop fully rescues the explosion

Running H30r1 with `UseDeltaPositionTracking=true` but **without**
`UseBiasOnlyContacts` (so the v2-NGS pass is still active):

```
Late-window peak: bias-only=75.53  NGS-backstop=4.03

step  bias-only per-box v               NGS per-box v
 200  8.9  9.0  0.1  0.0  0.6          1.1  0.1  0.2  0.6  0.6
 400  42.2 42.4  0.0  0.1  0.6         0.1  0.3  0.1  0.4  0.6
 599  75.4 75.5  0.1  0.0  0.6         1.1  0.5  0.3  0.6  0.6
```

**The NGS pass is doing real corrective work that the soft-spring bias
cannot replicate at intermediate tunings.** This is an important
architectural data point for the eventual flag flip — it means
bias-only is NOT a universal replacement for NGS; it's an equivalent
*at properly-tuned settings* but loses its safety net at others.

### Conclusion: documented limitation, not a code bug

The bias-only path has resonant (Hz, ratio) regions where rocking-
asymmetric contact manifolds + friction coupling produce energy growth
without an NGS backstop. Two safe corners exist in the (Hz, ratio)
space:

- **H120r1 (our current default)** — works because Hz is high enough
  that the resonance period is below scene timescales
- **H30r10 (cpp v3 default)** — works because damping is critical
  (no spring oscillation to resonate)

Intermediate values can hit resonance. Recommendation: only use those
two safe corners with bias-only mode. Users wanting to tune Hz/ratio
between them should either keep NGS active (don't flip
`UseBiasOnlyContacts` on) or stay close to the safe corners.

### Future architectural option (out of scope for today)

A potential cause-#7 fix would be a *contingent* NGS pass: only trigger
the position-constraint NGS when soft-bias correction over the
velocity-solve iterations fails to reduce penetration below a threshold.
That would keep cpp v3's bias-only speed on most contacts but engage
the NGS safety net when soft-spring tuning hits resonance. Not pursued
now — current safe corners are adequate and the architecture would
need careful design.
