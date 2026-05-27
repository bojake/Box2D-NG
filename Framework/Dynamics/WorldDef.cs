using System;

namespace Box2DNG
{
    public sealed class WorldDef
    {
        public Vec2 Gravity { get; private set; } = new Vec2(0f, -10f);
        public float RestitutionThreshold { get; private set; } = 1f;
        public float HitEventThreshold { get; private set; } = 1f;
        // Kept at legacy (120, 1) after the 2026-05-25 partial Phase 2.5 flip.
        // cpp v3's (30, 10) is the right tuning for the bias-only path
        // (BASELINE.md "Cause #3 reframed") but the NGS pass — which stays
        // active until `UseBiasOnlyContacts` is flipped on per-world — is
        // tuned for (120, 1). The contact-tuning flip moves together with
        // `UseBiasOnlyContacts` in a future commit, after per-sample
        // validation against Cantilever / Car / Breakable / FrictionJoint
        // (the regressions the bias-only flip exposed).
        public float ContactHertz { get; private set; } = 120f;
        public float ContactDampingRatio { get; private set; } = 1f;
        public float ContactSpeed { get; private set; } = 15f;
        public float MaximumLinearSpeed { get; private set; } = 1000f;
        public float MaximumAngularSpeed { get; private set; } = 50f;
        public float MaximumTranslation { get; private set; } = 2f;
        public float MaximumRotation { get; private set; } = 0.5f * MathF.PI;
        public bool EnableSleep { get; private set; } = true;
        public bool EnableContinuous { get; private set; } = true;
        // Phase 3 of TIER4_PARITY_PLAN: switch CCD from the legacy per-contact
        // ProcessTOI loop (which sub-steps each contact and pumps energy into
        // joint-coupled bodies — see TIER3_NOTES.md "TOI sub-stepping pumps
        // energy in joint-coupled chains") to a per-body sweep matching cpp
        // v3's `b2SolveContinuous`. Default false — the 2026-05-25 coordinated
        // Phase 2.5 flip kept the delta+bias+sub-step trio but NOT this. Per-
        // body CCD alone (without the delta-position machinery providing
        // within-step contact discovery for sub-steps) catastrophically
        // regresses CircleStress (-3290 y fall-through), Cantilever (8
        // bodies fell through), FrictionJoint, SliderCrank. The bullet-test
        // case is covered by `Sweep`-tracking fix in IntegratePositions
        // (Sweep now spans the full outer step, so legacy ProcessTOI still
        // catches tunneling under sub-stepping). Future flip needs a probe
        // pass before changing this default.
        public bool UsePerBodyCCD { get; private set; }
        // Phase 2.5 Stages B–K of TIER4_PARITY_PLAN: cpp v3's delta-position
        // model where `_bodyPositions[id]` / `_bodyRotations[id]` stay at
        // the outer-step start throughout the sub-step loop and
        // `_bodyDeltaPositions[id]` / `_bodyDeltaRotations[id]` carry the
        // within-step movement. `ApplyBodyDeltas` commits delta back into
        // the step-start arrays once per outer Step. Joint Solve methods +
        // contact NGS read `step-start + delta` for the "current effective"
        // pose. Flipped on 2026-05-25 alongside the rest of the coordinated
        // Phase 2.5 flip.
        public bool UseDeltaPositionTracking { get; private set; } = true;
        // Phase 2.5 cause #2: when true, `SolvePositionConstraints` skips the
        // v2-style position-constraint NGS pass for contacts and relies
        // entirely on the soft-contact bias that `ContactSolver` computes
        // when `EnableContactSoftening` is true. cpp box2d v3 uses this
        // bias-only model exclusively — its stacks settle from the bias
        // signal alone, no NGS backstop. Flipped on 2026-05-25 alongside
        // the rest of the coordinated Phase 2.5 flip. NOTE: bias-only is
        // tuning-sensitive (task #84); ContactHertz=30, ratio=10 must be
        // set together. Intermediate (Hz, ratio) tunings can produce
        // friction-bias coupling explosions (see BASELINE.md task #84).
        public bool UseBiasOnlyContacts { get; private set; } = false;
        public bool EnableContactSoftening { get; private set; } = true;
        public bool UseSoftConstraints { get; private set; } = true;
        public bool EnableContactHertzClamp { get; private set; }
        public bool EnableContactSolverSimd { get; private set; }
        // Step 4 of the 2026-05-26 cpp v3 pipeline refactor: when true, the
        // Relax pass also iterates joints with useBias=false (matches cpp's
        // b2_stageRelax calling b2SolveJointsTask). Disabled by default —
        // empirically un-does the Solve pass's Baumgarte limit recovery
        // (our `Cdot + C` form is aggressive and Relax's bias=0 form
        // counteracts it). Enable explicitly once joint limits are ported
        // to the cpp v3 softness form. Joint Solve methods all accept the
        // useBias parameter today regardless of this flag (Step 4a) — the
        // flag only controls whether Relax invokes them.
        public bool EnableJointRelax { get; private set; }
        public int MaxSubSteps { get; private set; } = 16;
        // cpp box2d v3's `ITERATIONS` = 1: a single velocity-solve pass per
        // sub-step. The historical default of 12 (Box2D 2.x lineage) carried
        // the burden of position correction *and* friction resolution in one
        // big iteration count; cpp's pipeline factors those out — bias drives
        // position correction in a single pass, `RelaxIterations` handles
        // residual velocity, and friction lives in the Relax pass. Flipped
        // to 1 (was 12) in Step 2 of the 2026-05-26 cpp v3 pipeline refactor
        // (HANDOFF.md). Tests that depend on the old count can opt back in
        // via `WithVelocityIterations(12)`.
        public int VelocityIterations { get; private set; } = 1;
        public int PositionIterations { get; private set; } = 6;
        // cpp box2d v3 sub-step pipeline has a `Relax` stage after
        // `IntegratePositions`: a few extra velocity-solve iterations with
        // `useBias = false`. This cleans residual velocity that the bias-
        // driven solve introduces without re-applying position correction.
        // Without it, bias-driven impulses can leave bodies with residual
        // downward velocity that accumulates step-over-step into penetration
        // the per-body CCD can't catch (task #86).
        //
        // Flipped 1 (was 0) in Step 2 of the 2026-05-26 cpp v3 pipeline
        // refactor (HANDOFF.md), coordinated with VelocityIterations 12→1
        // and the friction-into-Relax move (Step 3) — friction now runs
        // only in the useBias=false branch, so RelaxIterations must be > 0
        // for friction to resolve at all. Matches cpp's `RELAX_ITERATIONS`.
        public int RelaxIterations { get; private set; } = 1;
        // Internal sub-step count for the velocity-solve / position-integrate
        // loop. Phase 2 of TIER4_PARITY_PLAN. Each outer Step(timeStep) runs
        // the solver N times with h = timeStep/N — soft constraints (Phase 1)
        // benefit because their `b2MakeSoft(hertz, ratio, h)` becomes stiffer
        // per sub-step. Default 1 (kept after the 2026-05-25 partial flip).
        // The probe data (BASELINE.md "Cause #3 reframed") showed N=4 with
        // H30r10 + bias-only gives excellent Pyramid settling, but real-suite
        // testing with N=4 exposes catastrophic regressions in Cantilever /
        // FrictionJoint / SliderCrank / Car samples that the probes missed.
        // cpp v3 default is 4 — future work needs per-sample CCD-interaction
        // validation before flipping. Opt in per-test or per-sample.
        public int SubStepCount { get; private set; } = 1;
        public float JointForceThreshold { get; private set; } = float.MaxValue;
        public float JointTorqueThreshold { get; private set; } = float.MaxValue;
        // Global soft-constraint default for joints whose own Hertz is 0 (rigid).
        // Matches cpp box2d v3's per-world `b2_constraintSoftness` ([solver.h:170](../box2d-cpp/src/solver.h:170)).
        // Default (0, 0) preserves the legacy split-impulse / hard-constraint
        // behaviour for joints that don't opt into Hertz-driven stiffness.
        public float JointHertz { get; private set; }
        public float JointDampingRatio { get; private set; }
        public int WorkerCount { get; private set; } = 1;
        public int ArenaCapacity { get; private set; } = 1024 * 1024;
        public object? UserData { get; private set; }

        public Func<float, ulong, float, ulong, float>? FrictionCallback { get; private set; }
        public Func<float, ulong, float, ulong, float>? RestitutionCallback { get; private set; }

        public WorldDef WithGravity(Vec2 gravity) { Gravity = gravity; return this; }
        public WorldDef WithRestitutionThreshold(float value) { RestitutionThreshold = value; return this; }
        public WorldDef WithHitEventThreshold(float value) { HitEventThreshold = value; return this; }
        public WorldDef WithContactHertz(float hertz) { ContactHertz = hertz; return this; }
        public WorldDef WithContactDamping(float ratio) { ContactDampingRatio = ratio; return this; }
        public WorldDef WithContactSpeed(float speed) { ContactSpeed = speed; return this; }
        public WorldDef WithMaximumLinearSpeed(float speed) { MaximumLinearSpeed = speed; return this; }
        public WorldDef WithMaximumAngularSpeed(float speed) { MaximumAngularSpeed = speed; return this; }
        public WorldDef WithMaximumTranslation(float value) { MaximumTranslation = value; return this; }
        public WorldDef WithMaximumRotation(float value) { MaximumRotation = value; return this; }
        public WorldDef EnableSleeping(bool enable) { EnableSleep = enable; return this; }
        public WorldDef EnableContinuousCollision(bool enable) { EnableContinuous = enable; return this; }
        public WorldDef UsePerBodyContinuous(bool enable = true) { UsePerBodyCCD = enable; return this; }
        public WorldDef UseDeltaPositions(bool enable = true) { UseDeltaPositionTracking = enable; return this; }
        public WorldDef WithBiasOnlyContacts(bool enable = true) { UseBiasOnlyContacts = enable; return this; }
        public WorldDef EnableSoftening(bool enable) { EnableContactSoftening = enable; return this; }
        public WorldDef UseSoftConstraintsSolver(bool enable) { UseSoftConstraints = enable; return this; }
        public WorldDef EnableContactHertzClamping(bool enable) { EnableContactHertzClamp = enable; return this; }
        public WorldDef EnableContactSolverSimdPath(bool enable) { EnableContactSolverSimd = enable; return this; }
        public WorldDef WithJointRelax(bool enable = true) { EnableJointRelax = enable; return this; }
        public WorldDef WithMaxSubSteps(int steps) { MaxSubSteps = Math.Max(1, steps); return this; }
        public WorldDef WithVelocityIterations(int iterations) { VelocityIterations = Math.Max(1, iterations); return this; }
        public WorldDef WithPositionIterations(int iterations) { PositionIterations = Math.Max(1, iterations); return this; }
        public WorldDef WithRelaxIterations(int iterations) { RelaxIterations = Math.Max(0, iterations); return this; }
        public WorldDef WithSubStepCount(int count) { SubStepCount = Math.Max(1, count); return this; }
        public WorldDef WithJointForceThreshold(float threshold) { JointForceThreshold = Math.Max(0f, threshold); return this; }
        public WorldDef WithJointTorqueThreshold(float threshold) { JointTorqueThreshold = Math.Max(0f, threshold); return this; }
        public WorldDef WithJointHertz(float hertz) { JointHertz = Math.Max(0f, hertz); return this; }
        public WorldDef WithJointDampingRatio(float ratio) { JointDampingRatio = Math.Max(0f, ratio); return this; }
        public WorldDef WithWorkerCount(int count) { WorkerCount = Math.Max(1, count); return this; }
        public WorldDef WithArenaCapacity(int bytes) { ArenaCapacity = Math.Max(0, bytes); return this; }
        public WorldDef WithUserData(object? data) { UserData = data; return this; }
        public WorldDef WithFrictionCallback(Func<float, ulong, float, ulong, float>? callback)
        {
            FrictionCallback = callback;
            return this;
        }
        public WorldDef WithRestitutionCallback(Func<float, ulong, float, ulong, float>? callback)
        {
            RestitutionCallback = callback;
            return this;
        }
    }
}
