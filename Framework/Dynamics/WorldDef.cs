using System;

namespace Box2DNG
{
    public sealed class WorldDef
    {
        public Vec2 Gravity { get; private set; } = new Vec2(0f, -10f);
        public float RestitutionThreshold { get; private set; } = 1f;
        public float HitEventThreshold { get; private set; } = 1f;
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
        // v3's `b2SolveContinuous`. Default false during validation; flip to
        // true once all viewer samples settle cleanly with the new path.
        public bool UsePerBodyCCD { get; private set; }
        // Phase 2.5 Stages B–K of TIER4_PARITY_PLAN: opt-in to cpp v3's
        // delta-position model where `_bodyPositions[id]` / `_bodyRotations[id]`
        // stay at the outer-step start throughout the sub-step loop and
        // `_bodyDeltaPositions[id]` / `_bodyDeltaRotations[id]` carry the
        // within-step movement. `ApplyBodyDeltas` commits delta back into
        // the step-start arrays once per outer Step. Joint Solve methods +
        // contact NGS read `step-start + delta` for the "current effective"
        // pose. Default false during the consumer migration — the suite
        // stays byte-identical to Stage A.2 with the flag off. Flip to true
        // in dedicated tests as each consumer is migrated; flip the default
        // once all consumers are green at flag-on.
        public bool UseDeltaPositionTracking { get; private set; }
        // Phase 2.5 cause #2 seed: when true, `SolvePositionConstraints`
        // skips the v2-style position-constraint NGS pass for contacts and
        // relies entirely on the soft-contact bias that `ContactSolver`
        // computes when `EnableContactSoftening` is true. cpp box2d v3
        // uses this bias-only model exclusively — its stacks settle from
        // the bias signal alone, no NGS backstop. Our codebase has both
        // paths active by default; the NGS pass over-corrects when paired
        // with sub-stepping + soft springs, blocking the `SubStepCount > 1`
        // win the Phase 2.5 plan predicted. Flipping this flag on lets a
        // sample run the cpp v3 path and lets future work tune
        // `ContactHertz` / `ContactDampingRatio` for adequate settling
        // without the NGS backstop. Default false (no behaviour change).
        public bool UseBiasOnlyContacts { get; private set; }
        public bool EnableContactSoftening { get; private set; } = true;
        public bool UseSoftConstraints { get; private set; } = true;
        public bool EnableContactHertzClamp { get; private set; }
        public bool EnableContactSolverSimd { get; private set; }
        public int MaxSubSteps { get; private set; } = 16;
        public int VelocityIterations { get; private set; } = 12;
        public int PositionIterations { get; private set; } = 6;
        // Internal sub-step count for the velocity-solve / position-integrate
        // loop. Phase 2 of TIER4_PARITY_PLAN. Each outer Step(timeStep) runs
        // the solver N times with h = timeStep/N — soft constraints (Phase 1)
        // benefit because their `b2MakeSoft(hertz, ratio, h)` becomes stiffer
        // per sub-step. Default 1 preserves the legacy single-step behaviour
        // (byte-identical to pre-Phase-2 baseline); cpp default is 4.
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
        public WorldDef WithMaxSubSteps(int steps) { MaxSubSteps = Math.Max(1, steps); return this; }
        public WorldDef WithVelocityIterations(int iterations) { VelocityIterations = Math.Max(1, iterations); return this; }
        public WorldDef WithPositionIterations(int iterations) { PositionIterations = Math.Max(1, iterations); return this; }
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
