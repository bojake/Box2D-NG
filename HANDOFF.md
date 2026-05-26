# Handoff: cpp v3 Pipeline Refactor for Per-Body CCD

**Last session ended:** 2026-05-26
**Current task:** #86 — Make `UsePerBodyCCD` viable as default
**Status:** Architectural gap identified, partial plumbing landed. Full refactor remains.

## Read these first (in order)

1. **`BASELINE.md`** — full context of the Phase 2.5 work. Look especially at:
   - "Task #85: partial Phase 2.5 default flip" — what shipped, what didn't, why
   - "Task #86 progress" — diagnosis of why per-body CCD fails
2. **`TIER4_PARITY_PLAN.md`** — the original 5-phase plan + "What's different from cpp v3" table
3. **`SOLVER.md`** — current pipeline reference, especially the "What's different from cpp v3" table

## Where we are right now

Commits to look at (in reverse chronological order, most recent first):

| Hash | Summary |
|------|---------|
| `80cf9ae` | Relax phase plumbing + useBias=false softness fix in ContactSolver |
| `69e4259` | Per-body CCD diagnostic findings (Tests/PerBodyCCDDiagnosticProbe.cs) |
| `5c2eec5` | Partial Phase 2.5 default flip (UseDeltaPositionTracking=true) + Sweep CCD bug fix |
| `e47949b` | SampleCatalog.Factories — fixes singleton state leak in probes |
| `6486a56` | Task #80 closed: no bias-only regressions, cpp v3 defaults best |
| `5134835` | Task #82 closed: CircleStress regression is cause #2 |
| `d515c10` | Task #80 seed: UseBiasOnlyContacts flag + sample-by-sample probe |

**Suite state:** 450/450 passing at default `RelaxIterations=0` (no behaviour change from pre-refactor).

## The architectural diagnosis

Per-body CCD (Phase 3) catastrophically regresses CircleStress / Cantilever / Pyramid /
Dominos when enabled. Root cause is NOT a CCD detection bug — it's that our solver pipeline
diverges from cpp v3 in ways the legacy `ProcessTOI` was implicitly compensating for.

The gap, item by item:

| Aspect | cpp v3 (target) | Box2D-NG (current) |
|--------|-----------------|-------------------|
| Solve iterations (per sub-step) | `ITERATIONS = 1` | `VelocityIterations = 12` |
| Relax iterations | `RELAX_ITERATIONS = 1` | `RelaxIterations` (default 0; new flag) |
| Friction in Solve phase | NO | YES (every iteration) |
| Friction in Relax phase | YES | (Relax runs friction too via same solver) |
| Softness in `useBias=false` | OFF (`massScale=1, impulseScale=0`) | Now OFF (fixed in `80cf9ae`) |
| Softness representation | `b2Softness {BiasRate, MassScale, ImpulseScale}` | Single `gamma` scalar in `ContactConstraintPoint.Softness` |
| NGS position pass for contacts | None — bias-only | Active (toggleable via `UseBiasOnlyContacts`) |
| Per-body CCD trigger location | Inside `b2FinalizeBodiesTask` per body | Separate pass in `FinalizeStep` |
| Joint relax | YES (joints participate in Relax) | NO (Relax is contact-only) |

The chaotic RelaxIterations sweep on Pyramid+CCD proves the iteration-count mismatch:
```
relax: 0    1    2    3    4    5    6
FT:    809  176  0    320  598  0    0
```

Pyramid is fixable at relax=2 OR relax≥5 specifically. Other samples have different
sweet spots. There is NO RelaxIterations value that fixes all four problem scenes —
because the issue is structural, not parametric.

## The refactor (in dependency order)

### Step 1: Plumb cpp's `Softness(BiasRate, MassScale, ImpulseScale)` into ContactConstraintPoint

**Goal:** Replace `ContactConstraintPoint.Softness` (single `gamma` float) with
the full `Softness` struct so cpp's `useBias=false` semantics (`massScale=1,
impulseScale=0`) work the same way joints already use.

**Files:**
- `Framework/Common/Softness.cs` — struct already exists from Phase 1 (BiasRate,
  MassScale, ImpulseScale). Reuse it.
- `Framework/Dynamics/World.ContactSolver.cs` — `ContactConstraintPoint` struct
  (search for `public float Softness;` around line 733) needs to change to
  `public Softness Softness;`
- `Framework/Dynamics/World.cs` — `ComputeContactSoftness` (line 6639) needs to
  compute and return a `Softness` struct, not separate `bias` + `softness` floats.
  Today's `gamma = 1/(h*(d+h*k))` → cpp's `impulseScale = gamma` (similar but
  let `Softness.Make(hertz, ratio, h)` produce the canonical values).
- `Framework/Dynamics/World.ContactSolverSimd.cs` — parallel SIMD copies of the
  same code (search for `Bias = bias, Softness = softness` around lines 164, 355).

**Constraint equation change** in `SolveVelocityConstraint`:
```csharp
// Today (line 562-575):
float bias = useBias ? cp.Bias : 0f;
float softnessCoupling = useBias ? cp.Softness * cp.NormalImpulse : 0f;
float impulse = -(vn + bias + softnessCoupling) * cp.NormalMass;

// After:
float bias = useBias ? cp.Bias : 0f;  // velocityBias in cpp
float massScale = useBias ? cp.Softness.MassScale : 1f;
float impulseScale = useBias ? cp.Softness.ImpulseScale : 0f;
float impulse = -cp.NormalMass * (massScale * vn + bias) - impulseScale * cp.NormalImpulse;
```

Match cpp's `contact_solver.c:322-323`.

**Validation:** With `RelaxIterations=0` the suite should stay 450/450. Some
numbers may shift at the ulp level but no test should fail. If a test fails,
the new Softness math probably differs from the old gamma-based formula in a
numerically-significant way — debug by running `BaselineRecorder` with
`B2_BASELINE=1` env var and comparing tables.

### Step 2: Reduce VelocityIterations 12 → 1 (cpp's ITERATIONS)

**Goal:** Structurally match cpp's iteration count so Relax iterations carry
the weight cpp expects them to.

**Files:**
- `Framework/Dynamics/WorldDef.cs` — change `VelocityIterations` default 12 → 1.
- Re-calibrate sample test thresholds. Many will move; most should improve.

**Validation:** Re-run BaselineRecorder. Update SampleSettlingTests /
SampleActiveTests thresholds where physics improved. Look for regressions in
Dominos (friction-iteration-count sensitive), Bridge, Chain.

### Step 3: Move friction out of Solve, into Relax only

**Goal:** cpp resolves friction ONLY in the `useBias=false` (Relax) branch
(contact_solver.c:344-371). Our friction runs every iteration. After Step 2
reduces VelocityIterations to 1, this matters less, but matching cpp exactly
keeps the architecture clean.

**Files:**
- `Framework/Dynamics/World.ContactSolver.cs` — wrap the friction block (line
  581 onwards) in `if (!useBias) { ... }`. (We TRIED this in task #86 and it
  worked for Pyramid but broke Dominos OFF; that was because VelocityIterations
  was still 12. After Step 2's change it should be fine.)
- Same change in `Framework/Dynamics/World.ContactSolverSimd.cs`.

**Validation:** Pyramid + CCD + RelaxIterations=1 should now hit FT=0 cleanly.

### Step 4: Add joint relax — `useBias` parameter on all 10 joint Solve methods

**Goal:** cpp's Relax phase solves BOTH joints and contacts with `useBias=false`.
Currently we only relax contacts.

**Files:**
- Each `World.{Weld,Revolute,Prismatic,Wheel,Distance,Pulley,Rope,Gear,Motor,Friction}Joint.cs`
  needs a `useBias` parameter on its `Solve*JointVelocityConstraints` method.
- When `useBias=false`, skip the bias term (e.g., `linearBias = 0`,
  `angularBias = 0`) AND set `linearMassScale=1, linearImpulseScale=0` (and
  same for angular).
- `World.SolverPipeline.cs` — `SolveJointVelocityConstraints` dispatch needs a
  `useBias` parameter passed through.
- The new `SolveRelaxVelocityConstraints` in `SolverPipeline` should also call
  the joint solve with `useBias=false`.

**Validation:** Cantilever (rigid weld chain + per-body CCD) should now settle.

### Step 5: Re-calibrate all 35 sample test thresholds

**Goal:** After the pipeline change, all per-sample lateV/peakV thresholds
will have shifted. Re-record.

**Process:**
1. Run `B2_BASELINE=1 dotnet test --filter "FullyQualifiedName~BaselineRecorder"`
   for each configuration (default, +per-body-CCD, +bias-only, +sub-step).
2. Paste the new tables into BASELINE.md under a new dated section.
3. Update `SampleSettlingTests.cs`, `SampleActiveTests.cs`, etc. with new
   thresholds. Where physics improved (lower lateV / fewer fall-throughs),
   TIGHTEN the thresholds — the plan ([TIER4_PARITY_PLAN.md] line 14) explicitly
   says to do this.

### Step 6: Flip the defaults

Once Steps 1-5 are done and all samples are clean:
- `WorldDef.RelaxIterations`: 0 → 1
- `WorldDef.UsePerBodyCCD`: false → true
- `WorldDef.SubStepCount`: 1 → 4 (now safe with full cpp pipeline)
- `WorldDef.UseBiasOnlyContacts`: false → true (now safe with relax handling)
- `WorldDef.ContactHertz`: 120 → 30
- `WorldDef.ContactDampingRatio`: 1 → 10

This is the "second half" of task #85 that we deferred. Once it lands:
- Delete legacy `ProcessTOI` / `IntegrateForTOI` / per-contact sub-step (~250 LOC).
- Drop Cantilever's stiffer-than-cpp 30Hz spring tune (revert to cpp's 15Hz).
- Close tasks #85, #86. Phase 3 + Phase 4 complete.

## Reference: cpp v3 source pointers

All under `../box2d-cpp/src/`. **Available locally** — confirm with `ls ../box2d-cpp/src/`.

| File:Line | What's there |
|-----------|--------------|
| `solver.c:28-29` | `#define ITERATIONS 1` and `RELAX_ITERATIONS 1` |
| `solver.c:113-161` | `b2IntegratePositionsTask` — clamp + write deltaPosition |
| `solver.c:379-541` | `b2SolveContinuous` — per-body CCD impl, called from b2FinalizeBodiesTask |
| `solver.c:575-665` | `b2FinalizeBodiesTask` — where center += delta, where CCD is invoked |
| `solver.c:1077-1153` | The main sub-step loop (Solve → IntegratePositions → Relax) |
| `contact_solver.c:239-410` | `b2SolveContacts_Overflow` — useBias branch + friction only in useBias=false |
| `contact_solver.c:295-340` | The exact useBias=true vs useBias=false math for normal impulse |
| `solver.h:127` | `b2Softness` struct (BiasRate, MassScale, ImpulseScale) |
| `solver.h:239` | `b2MakeSoft` — the soft-constraint constructor |

## Gotchas (things that bit me last session)

1. **`SampleCatalog.All` has singleton state leak.** Use `SampleCatalog.Factories`
   for any probe that calls `Build()` multiple times per sample. See
   `Tests/SampleCatalogFactoriesTests.cs` for the regression test and
   `BASELINE.md` "Task #83" for the diagnosis.

2. **Don't increase RelaxIterations default before Steps 1-3 land.** I tried
   `RelaxIterations=1` with VelocityIterations=12 and it broke Pyramid OFF
   (FT 0→34) and Dominos OFF (FT 0→23). The full architecture has to move
   together.

3. **Stage K's Sweep collapse in `SolvePositionConstraints` (World.cs line
   6986)** was added to fix an AddPair hang via legacy ProcessTOI. Don't
   remove it without re-validating AddPair — I tried in task #86 and the
   suite hung. The interaction with per-body CCD is documented in-code at
   that line.

4. **The Sweep tracking fix in `IntegratePositions`** (snapshot before
   `body.SetTransform`, preserve C0/A0/Alpha0 across sub-steps) is a real
   bug fix from task #85. Don't lose it.

5. **CompoundShapes is the one scene where NGS-with-delta-tracking is
   meaningfully better than bias-only**, at any (Hz, ratio) tuning. After
   Step 6 flips bias-only on, CompoundShapes lateV goes from 14.61 → 39.06.
   Either accept as a documented trade-off, design contingent NGS, or
   per-scene tune. See `BASELINE.md` "Task #83: fixture leak fix —
   corrected numbers".

6. **VaryingFriction is highly tuning-sensitive under bias-only.** 8 of 16
   (Hz, ratio) grid cells explode. cpp's H30r10 is critically overdamped
   and safe; intermediate values aren't. Stay near safe corners.

## Quick verification checklist for the new session

Before starting work, confirm baseline state:

```bash
# 1. Verify suite passes at current state (should be 450/450)
dotnet test Tests/box2d-NG.Tests.csproj --no-build \
  --filter "FullyQualifiedName!~Probe&FullyQualifiedName!~RecordAllSampleMetrics" \
  --logger "console;verbosity=minimal"

# 2. Verify Pyramid + CCD + relax=2 hits FT=0 (sanity check Relax phase plumbing)
# (Manually set RelaxIterations=2 via WithRelaxIterations in a probe.)

# 3. Confirm box2d-cpp source available
ls ../box2d-cpp/src/solver.c ../box2d-cpp/src/contact_solver.c

# 4. Skim TIER4_PARITY_PLAN.md "Current state vs cpp v3" table
```

## Estimated effort

- **Step 1** (Softness struct plumbing): 2-4 hours. Mostly mechanical.
- **Step 2** (VelocityIterations 12→1): 30 min code change, 1-2 hours threshold updates.
- **Step 3** (friction in Relax only): 30 min code, depends on Step 2.
- **Step 4** (joint relax): 4-6 hours. 10 joint files, each ~30 min.
- **Step 5** (re-calibrate): 2-4 hours running + updating thresholds.
- **Step 6** (flip defaults): 30 min, plus validation.

**Total: ~1.5-2 days** of focused work. Aligns with the original TIER4_PARITY_PLAN
estimate of "1 week" for Phase 4 cleanup work.

## Existing scaffolding to leverage

- `Tests/PerBodyCCDDiagnosticProbe.cs` — already validates the per-body CCD
  failure pattern. Should still surface FT > 0 after Steps 1-3 but FT=0 after
  Step 4 (with `RelaxIterations=1`).
- `Tests/FlagOnSampleProbe.cs` — full-catalog sweep. Useful for Step 5.
- `Tests/BiasOnlyRetuneProbe.cs` — has the (Hz, ratio) grid sweep that
  generated cpp's H30r10 default recommendation.
- `Tests/BaselineRecorder.cs` — opt-in via `B2_BASELINE=1` env var. Use for
  Step 5 to dump new tables.
- `Framework/Common/Softness.cs` — the `b2Softness`-equivalent struct already
  exists (BiasRate, MassScale, ImpulseScale). Joints use it. Just need to
  plumb into contacts (Step 1).

## If you get stuck

The chaotic RelaxIterations sweep in task #86 demonstrates that **adding pieces
of cpp's pipeline independently doesn't compose well** — each piece relies on
the others. Resist the temptation to land Steps 1-4 individually and validate.
Instead:

1. Land Step 1 (mechanical, byte-identical to current at RelaxIterations=0).
2. Land Steps 2-4 together as one commit. Re-calibration is the bulk of the
   work; doing it incrementally means re-calibrating 4× instead of 1×.
3. Step 5 happens during Step 2-4 development.
4. Step 6 is the final flip, separate commit.

Don't be afraid to revert if a step breaks more than expected — the
investigation probes survived all my reverts and will keep surfacing issues
for the next attempt.
