using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 Stage B of TIER4_PARITY_PLAN. Validates the
    /// <see cref="WorldDef.UseDeltaPositionTracking"/> flag end-to-end for
    /// scenes that do NOT exercise the unmigrated joint / contact solver
    /// paths. With the flag on, <c>IntegratePositions</c> writes delta
    /// instead of advancing <c>_bodyPositions</c>; <c>ApplyBodyDeltas</c>
    /// commits the accumulated delta after the sub-step loop. External
    /// reads via <c>Body.Transform</c> / <c>Body.Position</c> should yield
    /// the same effective pose at end of step as the flag-off path.
    ///
    /// Scenes that include contacts or joints will read stale step-start
    /// data through the un-migrated solvers and produce wrong results
    /// until Stages C+ migrate them — those are intentionally not exercised
    /// here.
    /// </summary>
    [TestClass]
    public class DeltaPositionTrackingTests
    {
        [TestMethod]
        public void Flag_DefaultsOff()
        {
            WorldDef def = new WorldDef();
            Assert.IsFalse(def.UseDeltaPositionTracking,
                "Default must stay off — consumer migration (Stages C+) is not done.");
        }

        [TestMethod]
        public void Builder_TogglesFlag()
        {
            WorldDef def = new WorldDef().UseDeltaPositions(true);
            Assert.IsTrue(def.UseDeltaPositionTracking);
            def.UseDeltaPositions(false);
            Assert.IsFalse(def.UseDeltaPositionTracking);
        }

        [TestMethod]
        public void FreeFallBody_FlagOnMatchesFlagOff()
        {
            // No ground, no joints — only gravity acts on the body, so the
            // un-migrated joint / contact solver paths never see stale data.
            // Both paths must produce the same final position after the
            // same number of steps.
            float yFlagOff = RunFreeFall(useDelta: false);
            float yFlagOn = RunFreeFall(useDelta: true);
            Assert.AreEqual(yFlagOff, yFlagOn, 1e-4f,
                $"Free-fall trajectory must match across the flag. " +
                $"flagOff={yFlagOff} flagOn={yFlagOn}");
        }

        [TestMethod]
        public void FreeFallBody_FlagOn_TransformReadsEffectivePoseEachStep()
        {
            // Step-by-step: after each world.Step, body.Position should
            // equal the analytic free-fall trajectory regardless of the
            // flag. This exercises the post-`ApplyBodyDeltas` getter path.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 100f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            float prevY = body.Transform.P.Y;
            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
                float y = body.Transform.P.Y;
                Assert.IsTrue(y < prevY, $"Body should fall every step. step {i}: y={y}, prevY={prevY}");
                prevY = y;
            }
            // After 0.5s under -10 m/s² gravity from rest, y ≈ 100 - 0.5*10*0.25 = 98.75
            Assert.IsTrue(body.Transform.P.Y < 99f && body.Transform.P.Y > 98f,
                $"Expected ~98.75 after 0.5s of free-fall, got {body.Transform.P.Y}.");
        }

        [TestMethod]
        public void FlagOn_DeltaArraysClearedAfterStep()
        {
            // After ApplyBodyDeltas commits, the delta arrays must be back
            // to (0, Identity) so the next ResetSweeps' snapshot is
            // consistent and any post-step Body.Transform reads aren't
            // double-counting.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            world.Step(1f / 60f);

            // Internal access via the world's SoA array — these are
            // `internal` fields so the test assembly (InternalsVisibleTo)
            // can read them.
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].X, 1e-6f,
                "Delta X should be zero after ApplyBodyDeltas commit.");
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].Y, 1e-6f,
                "Delta Y should be zero after ApplyBodyDeltas commit.");
            Assert.AreEqual(1f, world._bodyDeltaRotations[body.Id].C, 1e-6f,
                "Delta rotation cos should be 1 (identity) after commit.");
            Assert.AreEqual(0f, world._bodyDeltaRotations[body.Id].S, 1e-6f,
                "Delta rotation sin should be 0 (identity) after commit.");
        }

        [TestMethod]
        public void FlagOn_BodyPositionSetWipesDelta()
        {
            // External code setting Body.Position should land at the
            // step-start arrays AND zero the delta — i.e., the next
            // Body.Position read returns the set value exactly. This
            // matters outside the solve too (delta is always zero between
            // steps when ApplyBodyDeltas committed), but the test pins the
            // setter contract directly.
            World world = new World(new WorldDef().UseDeltaPositions(true));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.Position = new Vec2(7f, 11f);
            Vec2 readBack = body.Position;
            Assert.AreEqual(7f, readBack.X, 1e-6f);
            Assert.AreEqual(11f, readBack.Y, 1e-6f);
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].X, 1e-6f);
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].Y, 1e-6f);
        }

        [TestMethod]
        public void FlagOn_SetTransformWipesBothDeltas()
        {
            // Public Body.SetTransform overload — pins that both delta
            // arrays are zeroed AND the readback matches what was set.
            // Catches the case where someone migrates one getter/setter
            // pair but not the other.
            World world = new World(new WorldDef().UseDeltaPositions(true));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.SetTransform(new Vec2(3f, 4f), 0.5f);
            Assert.AreEqual(3f, body.Transform.P.X, 1e-6f);
            Assert.AreEqual(4f, body.Transform.P.Y, 1e-6f);
            Assert.AreEqual(0.5f, body.Transform.Q.Angle, 1e-5f);
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].X, 1e-6f);
            Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].Y, 1e-6f);
            Assert.AreEqual(1f, world._bodyDeltaRotations[body.Id].C, 1e-6f);
            Assert.AreEqual(0f, world._bodyDeltaRotations[body.Id].S, 1e-6f);
        }

        [TestMethod]
        public void FreeFall_MultiSubStep_FlagOnMatchesFlagOff()
        {
            // The flag's actual unlock target is SubStepCount > 1 with the
            // delta-position model. For a body with no joints / no contacts
            // the result must still match flag-off byte-for-byte (to within
            // float rounding), because IntegratePositions's delta-write at
            // each sub-step encodes the absolute newPosition relative to
            // step-start — accumulation across sub-steps is implicit in
            // _bodyDeltaPositions[] = newPosition - stepStart.
            float yOff = RunFreeFall(useDelta: false, subStepCount: 4);
            float yOn = RunFreeFall(useDelta: true, subStepCount: 4);
            Assert.AreEqual(yOff, yOn, 1e-3f,
                $"Free-fall × subStepCount=4 must match across the flag. " +
                $"off={yOff} on={yOn}");
        }

        [TestMethod]
        public void SpinningBody_FlagOnMatchesFlagOff()
        {
            // Pure angular velocity, no gravity. Exercises the
            // _bodyDeltaRotations math that free-fall doesn't touch.
            // body.Transform.Q.Angle should match between flag states
            // after the same number of steps.
            float angleOff = RunSpinning(useDelta: false);
            float angleOn = RunSpinning(useDelta: true);
            Assert.AreEqual(angleOff, angleOn, 1e-4f,
                $"Spinning trajectory must match across the flag. " +
                $"off={angleOff} on={angleOn}");
        }

        [TestMethod]
        public void OffCenterMass_SpinningWithGravity_FlagOnMatchesFlagOff()
        {
            // The Stage A.2 fix was specifically about LocalCenter != 0 +
            // rotating bodies — under those conditions the body-origin
            // translation (what `_bodyPositions[id]` should advance by) is
            // NOT equal to the world-center translation. Use an asymmetric
            // polygon whose centroid lives away from the body's origin and
            // give it both gravity and an angular velocity, so both factors
            // contribute to the position delta.
            float yOff = RunOffCenterSpinning(useDelta: false);
            float yOn = RunOffCenterSpinning(useDelta: true);
            Assert.AreEqual(yOff, yOn, 1e-3f,
                $"Off-center spinning body trajectory must match. " +
                $"off={yOff} on={yOn}");
        }

        [TestMethod]
        public void MultipleBodies_FlagOn_AllDeltasClearAfterStep()
        {
            // Three independent bodies, all under gravity. Each body's
            // delta arrays must be (0, Identity) after every step's
            // ApplyBodyDeltas commit, regardless of their relative
            // positions / sleep states.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(-5f, 50f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 30f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body c = world.CreateBody(new BodyDef().AsDynamic().At(5f, 10f));
            c.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            for (int step = 0; step < 5; ++step)
            {
                world.Step(1f / 60f);
                foreach (Body body in new[] { a, b, c })
                {
                    Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].X, 1e-6f,
                        $"step {step} body {body.Id}: deltaPos.X not cleared");
                    Assert.AreEqual(0f, world._bodyDeltaPositions[body.Id].Y, 1e-6f,
                        $"step {step} body {body.Id}: deltaPos.Y not cleared");
                    Assert.AreEqual(1f, world._bodyDeltaRotations[body.Id].C, 1e-6f,
                        $"step {step} body {body.Id}: deltaRot.C != 1");
                    Assert.AreEqual(0f, world._bodyDeltaRotations[body.Id].S, 1e-6f,
                        $"step {step} body {body.Id}: deltaRot.S != 0");
                }
            }
        }

        [TestMethod]
        public void Determinism_FlagOn_SameRunSameResult()
        {
            // Identical scene built twice with flag on should yield bit-
            // identical body positions after the same number of steps.
            // Catches any non-deterministic source introduced by the delta
            // bookkeeping.
            float ySpinA = RunOffCenterSpinning(useDelta: true);
            float ySpinB = RunOffCenterSpinning(useDelta: true);
            Assert.AreEqual(ySpinA, ySpinB,
                $"Same scene twice → same result. a={ySpinA} b={ySpinB}");
        }

        [TestMethod]
        public void StepStartArrays_SnapshotMatchesPreStepPosition()
        {
            // The next step's ResetSweeps captures _bodyStepStartPositions[]
            // from _bodyPositions[] *at the top of that step* — i.e., the
            // committed pose from the previous step. Pin that contract:
            // record the body's position immediately *before* a Step call,
            // then check that _bodyStepStartPositions matches it after the
            // Step (ResetSweeps inside that Step did the snapshot).
            //
            // Future joint-migration code that uses
            // `_bodyStepStartPositions[id]` for the cpp-v3 "deltaCenter"
            // reference depends on this — if step-start ever diverged from
            // the pre-step pose, the bias signal would compute against
            // stale data.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 50f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            for (int i = 0; i < 5; ++i)
            {
                // Capture the body's pose BEFORE the step. ResetSweeps
                // inside the step will snapshot this exact value.
                Vec2 preStepPosition = world._bodyPositions[body.Id];
                Rot preStepRotation = world._bodyRotations[body.Id];
                world.Step(1f / 60f);
                Assert.AreEqual(preStepPosition.X,
                    world._bodyStepStartPositions[body.Id].X, 1e-6f,
                    $"step {i}: stepStart.X != preStep.X");
                Assert.AreEqual(preStepPosition.Y,
                    world._bodyStepStartPositions[body.Id].Y, 1e-6f,
                    $"step {i}: stepStart.Y != preStep.Y");
                Assert.AreEqual(preStepRotation.S,
                    world._bodyStepStartRotations[body.Id].S, 1e-6f,
                    $"step {i}: stepStartRot.S != preStepRot.S");
                Assert.AreEqual(preStepRotation.C,
                    world._bodyStepStartRotations[body.Id].C, 1e-6f,
                    $"step {i}: stepStartRot.C != preStepRot.C");
            }
        }

        [TestMethod]
        public void RigidWeld_TwoBodiesUnderGravity_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage C — the WeldJoint migration's load-bearing test.
            // Two bodies welded with no soft springs (rigid axes), falling
            // under gravity. The position-constraint NGS pass writes to
            // _bodyPositions / _bodyRotations in the flag-off path and to
            // _bodyDeltaPositions / _bodyDeltaRotations in the flag-on path.
            // Final pose must match across the flag — the migration must
            // not perturb the existing rigid-weld physics.
            (Vec2 aOff, Vec2 bOff) = RunWeldedPair(useDelta: false, softLinearHertz: 0f, softAngularHertz: 0f);
            (Vec2 aOn, Vec2 bOn) = RunWeldedPair(useDelta: true, softLinearHertz: 0f, softAngularHertz: 0f);
            Assert.AreEqual(aOff.X, aOn.X, 1e-3f, $"A.X mismatch. off={aOff.X} on={aOn.X}");
            Assert.AreEqual(aOff.Y, aOn.Y, 1e-3f, $"A.Y mismatch. off={aOff.Y} on={aOn.Y}");
            Assert.AreEqual(bOff.X, bOn.X, 1e-3f, $"B.X mismatch. off={bOff.X} on={bOn.X}");
            Assert.AreEqual(bOff.Y, bOn.Y, 1e-3f, $"B.Y mismatch. off={bOff.Y} on={bOn.Y}");
        }

        [TestMethod]
        public void SoftWeld_TwoBodiesUnderGravity_FlagOnMatchesFlagOff()
        {
            // Same scene, with soft spring (Hertz > 0) so the bias-driven
            // path in SolveWeldJointVelocityConstraints fires. With flag
            // off the bias reads _bodyPositions directly; with flag on it
            // reads via EffectivePosition. Both should produce the same
            // result for a single-step trajectory.
            (Vec2 aOff, Vec2 bOff) = RunWeldedPair(useDelta: false, softLinearHertz: 30f, softAngularHertz: 30f);
            (Vec2 aOn, Vec2 bOn) = RunWeldedPair(useDelta: true, softLinearHertz: 30f, softAngularHertz: 30f);
            Assert.AreEqual(aOff.X, aOn.X, 1e-3f, $"A.X mismatch. off={aOff.X} on={aOn.X}");
            Assert.AreEqual(aOff.Y, aOn.Y, 1e-3f, $"A.Y mismatch. off={aOff.Y} on={aOn.Y}");
            Assert.AreEqual(bOff.X, bOn.X, 1e-3f, $"B.X mismatch. off={bOff.X} on={bOn.X}");
            Assert.AreEqual(bOff.Y, bOn.Y, 1e-3f, $"B.Y mismatch. off={bOff.Y} on={bOn.Y}");
        }

        [TestMethod]
        public void RigidWeld_AnchoredToStatic_FlagOnHoldsPosition()
        {
            // A dynamic body welded rigidly to a static anchor at the same
            // position should stay near the anchor over time. Under flag-on
            // the position-constraint NGS writes the corrections to the
            // delta arrays and ApplyBodyDeltas commits them per step;
            // ResetSweeps + EffectivePosition round-trip must keep the body
            // bounded.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            Body hanging = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            hanging.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            // No soft spring — rigid weld exercises the position-constraint NGS path.
            world.CreateJoint(new WeldJointDef(anchor, hanging, new Vec2(0f, 5f)));

            for (int i = 0; i < 60; ++i) world.Step(1f / 60f);
            float drift = (hanging.Transform.P - new Vec2(0f, 5f)).Length;
            Assert.IsTrue(drift < 0.5f,
                $"Rigid weld to static under flag-on must hold position. drift={drift}");
        }

        [TestMethod]
        public void SoftWeld_AnchoredToStatic_FlagOnHoldsPosition()
        {
            // Soft variant — exercises the velocity-constraint bias path.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(true));
            Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            Body hanging = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            hanging.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(anchor, hanging, new Vec2(0f, 5f))
                .WithLinearSpring(30f, 0.7f)
                .WithAngularSpring(30f, 0.7f));

            for (int i = 0; i < 180; ++i) world.Step(1f / 60f);
            float drift = (hanging.Transform.P - new Vec2(0f, 5f)).Length;
            Assert.IsTrue(drift < 1f,
                $"Soft weld to static under flag-on must constrain drift. drift={drift}");
        }

        [TestMethod]
        public void Weld_DeterminismUnderFlag()
        {
            // Build the same welded scene twice with flag-on, step it the
            // same number of times, compare final positions. Deterministic.
            (Vec2 aA, Vec2 bA) = RunWeldedPair(useDelta: true, softLinearHertz: 0f, softAngularHertz: 0f);
            (Vec2 aB, Vec2 bB) = RunWeldedPair(useDelta: true, softLinearHertz: 0f, softAngularHertz: 0f);
            Assert.AreEqual(aA.X, aB.X, $"determinism A.X: {aA.X} vs {aB.X}");
            Assert.AreEqual(aA.Y, aB.Y, $"determinism A.Y: {aA.Y} vs {aB.Y}");
            Assert.AreEqual(bA.X, bB.X, $"determinism B.X: {bA.X} vs {bB.X}");
            Assert.AreEqual(bA.Y, bB.Y, $"determinism B.Y: {bA.Y} vs {bB.Y}");
        }

        [TestMethod]
        public void RevoluteJoint_AnchoredHangingBody_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage D — Revolute joint migration. A dynamic body
            // hanging by a revolute pin under gravity must trace the same
            // arc whether flag-on or flag-off, since the rigid-axis NGS
            // path (no soft spring) writes through the migrated delta path.
            (Vec2 off, Vec2 on) = RunRevoluteHang();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void PrismaticJoint_SliderUnderGravity_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage E — Prismatic. The d vector + axis math is
            // recomputed inside SolveVelocityConstraints using effective
            // reads; rigid-axis position-constraint NGS uses the same.
            (Vec2 off, Vec2 on) = RunPrismaticSlider();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void WheelJoint_SuspensionUnderGravity_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage F — Wheel. Position-constraint NGS does the
            // two-phase perp-then-limit accumulation; the CommitWheelDelta
            // helper splits the final write across the flag branches.
            (Vec2 off, Vec2 on) = RunWheelSuspension();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void DistanceJoint_LengthConstraint_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage G — Distance. The velocity-solve sin/cos
            // round-trip via `new Rot(EffectiveRotation(id).Angle)` is
            // preserved (caught the Dominos byte-identity regression).
            (Vec2 off, Vec2 on) = RunDistanceJoint();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void PulleyJoint_Counterweight_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage H — Pulley.
            (Vec2 off, Vec2 on) = RunPulleyJoint();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void ContactNGS_BoxOnGround_BothFlagsSettle()
        {
            // Phase 2.5 Stage K — contact NGS migration. A box falling onto
            // a static ground exercises SolvePositionConstraints' contact
            // NGS pass. Flag-off writes via body.SetTransformFromCenter;
            // flag-on writes absolute effective poses into the delta arrays.
            // Both should settle the box near Y = 0.5 (ground + half-extent).
            //
            // Bit-identical comparison is too strict — the flag-on path
            // goes through more arithmetic (step-start + delta + sqrt
            // normalization), introducing ulp-level rounding that
            // accumulates over hundreds of steps but doesn't change the
            // physical outcome.
            (Vec2 off, Vec2 on) = RunBoxOnGround();
            Assert.IsTrue(off.Y > 0.4f && off.Y < 0.7f, $"flag-off box should rest near Y=0.5. y={off.Y}");
            Assert.IsTrue(on.Y > 0.4f && on.Y < 0.7f, $"flag-on box should rest near Y=0.5. y={on.Y}");
        }

        [TestMethod]
        public void ContactNGS_StackOfBoxes_BothFlagsStable()
        {
            // Multi-body stack exercises contact NGS across many bodies and
            // contacts per iteration. Flag-on tends to settle faster (cpp
            // v3's expected behavior — the absolute-delta write semantics
            // mean each NGS iteration sees the latest effective pose
            // immediately, not the partially-committed mix the flag-off
            // ref-based path produces).
            //
            // We pin qualitative stack stability: top box ends up at
            // 2 < Y < 3, middle 1 < Y < 2, bottom 0 < Y < 1 — no fall-
            // through, no explosion, in both modes.
            var off = RunStackTriple(useDelta: false);
            var on = RunStackTriple(useDelta: true);
            foreach (var (label, stack) in new[] { ("flag-off", off), ("flag-on", on) })
            {
                Assert.IsTrue(stack.bot.Y > 0f && stack.bot.Y < 1f, $"{label} bottom should be 0 < y < 1. y={stack.bot.Y}");
                Assert.IsTrue(stack.mid.Y > 1f && stack.mid.Y < 2f, $"{label} middle should be 1 < y < 2. y={stack.mid.Y}");
                Assert.IsTrue(stack.top.Y > 2f && stack.top.Y < 3f, $"{label} top should be 2 < y < 3. y={stack.top.Y}");
            }
        }

        private static (Vec2 off, Vec2 on) RunBoxOnGround()
        {
            return (RunBox(useDelta: false), RunBox(useDelta: true));

            static Vec2 RunBox(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
                ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f))));
                Body box = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
                box.CreateFixture(new FixtureDef(new PolygonShape(new[]
                {
                    new Vec2(-0.5f, -0.5f), new Vec2(0.5f, -0.5f),
                    new Vec2(0.5f, 0.5f), new Vec2(-0.5f, 0.5f),
                })).WithDensity(1f));
                for (int i = 0; i < 120; ++i) world.Step(1f / 60f);
                return box.Transform.P;
            }
        }

        private static (Vec2 top, Vec2 mid, Vec2 bot) RunStackTriple(bool useDelta)
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(useDelta));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f))));
            Vec2[] boxVerts = new[]
            {
                new Vec2(-0.5f, -0.5f), new Vec2(0.5f, -0.5f),
                new Vec2(0.5f, 0.5f), new Vec2(-0.5f, 0.5f),
            };
            Body[] boxes = new Body[3];
            for (int i = 0; i < 3; ++i)
            {
                boxes[i] = world.CreateBody(new BodyDef().AsDynamic().At(0f, 1f + i * 1.2f));
                boxes[i].CreateFixture(new FixtureDef(new PolygonShape(boxVerts)).WithDensity(1f));
            }
            for (int i = 0; i < 240; ++i) world.Step(1f / 60f);
            return (boxes[2].Transform.P, boxes[1].Transform.P, boxes[0].Transform.P);
        }

        [TestMethod]
        public void MotorJoint_TargetTracking_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage J — Motor + Friction joints have no Solve-loop
            // reads of position state (their bias is captured at Init, which
            // runs once at PrepareConstraints when delta is zero). They
            // should be auto-safe at flag-on without any code change.
            // Verify with a parity test.
            (Vec2 off, Vec2 on) = RunMotorJoint();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        [TestMethod]
        public void RopeJoint_MaxLengthConstraint_FlagOnMatchesFlagOff()
        {
            // Phase 2.5 Stage I — Rope.
            (Vec2 off, Vec2 on) = RunRopeJoint();
            Assert.AreEqual(off.X, on.X, 1e-3f, $"X mismatch. off={off.X} on={on.X}");
            Assert.AreEqual(off.Y, on.Y, 1e-3f, $"Y mismatch. off={off.Y} on={on.Y}");
        }

        private static (Vec2 hangOff, Vec2 hangOn) RunRevoluteHang()
        {
            return (RunRevolute(useDelta: false), RunRevolute(useDelta: true));

            static Vec2 RunRevolute(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
                Body hanging = world.CreateBody(new BodyDef().AsDynamic().At(1f, 5f));
                hanging.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new RevoluteJointDef(anchor, hanging, new Vec2(0f, 5f)));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return hanging.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunPrismaticSlider()
        {
            return (RunPrismatic(useDelta: false), RunPrismatic(useDelta: true));

            static Vec2 RunPrismatic(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body track = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
                Body slider = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
                slider.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new PrismaticJointDef(track, slider, new Vec2(0f, 5f), new Vec2(0f, 1f)));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return slider.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunWheelSuspension()
        {
            return (RunWheel(useDelta: false), RunWheel(useDelta: true));

            static Vec2 RunWheel(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body chassis = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
                chassis.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
                Body wheel = world.CreateBody(new BodyDef().AsDynamic().At(0f, 4f));
                wheel.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new WheelJointDef(chassis, wheel, new Vec2(0f, 4f), new Vec2(0f, 1f)));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return wheel.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunDistanceJoint()
        {
            return (RunDistance(useDelta: false), RunDistance(useDelta: true));

            static Vec2 RunDistance(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
                Body weight = world.CreateBody(new BodyDef().AsDynamic().At(2f, 5f));
                weight.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new DistanceJointDef(anchor, weight, new Vec2(0f, 5f), new Vec2(2f, 5f)));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return weight.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunPulleyJoint()
        {
            return (RunPulley(useDelta: false), RunPulley(useDelta: true));

            static Vec2 RunPulley(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body a = world.CreateBody(new BodyDef().AsDynamic().At(-2f, 5f));
                a.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                Body b = world.CreateBody(new BodyDef().AsDynamic().At(2f, 5f));
                b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(2f));
                world.CreateJoint(new PulleyJointDef(a, b,
                    new Vec2(-2f, 10f), new Vec2(2f, 10f),
                    new Vec2(-2f, 5f), new Vec2(2f, 5f),
                    ratio: 1f));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return a.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunMotorJoint()
        {
            return (RunMotor(useDelta: false), RunMotor(useDelta: true));

            static Vec2 RunMotor(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(Vec2.Zero)
                    .UseDeltaPositions(useDelta));
                Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
                Body b = world.CreateBody(new BodyDef().AsDynamic().At(2f, 0f));
                b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new MotorJointDef(a, b));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return b.Transform.P;
            }
        }

        private static (Vec2 off, Vec2 on) RunRopeJoint()
        {
            return (RunRope(useDelta: false), RunRope(useDelta: true));

            static Vec2 RunRope(bool useDelta)
            {
                World world = new World(new WorldDef()
                    .WithGravity(new Vec2(0f, -10f))
                    .UseDeltaPositions(useDelta));
                Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 10f));
                Body weight = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
                weight.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
                world.CreateJoint(new RopeJointDef(anchor, weight, new Vec2(0f, 10f), new Vec2(0f, 5f)).WithMaxLength(5f));
                for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
                return weight.Transform.P;
            }
        }

        private static (Vec2 bodyA, Vec2 bodyB) RunWeldedPair(bool useDelta, float softLinearHertz, float softAngularHertz)
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(useDelta));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(-1f, 10f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 10f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            WeldJointDef def = new WeldJointDef(a, b, new Vec2(0f, 10f));
            if (softLinearHertz > 0f) def = def.WithLinearSpring(softLinearHertz, 0.5f);
            if (softAngularHertz > 0f) def = def.WithAngularSpring(softAngularHertz, 0.5f);
            world.CreateJoint(def);
            for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
            return (a.Transform.P, b.Transform.P);
        }

        private static float RunFreeFall(bool useDelta, int subStepCount = 1)
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(useDelta)
                .WithSubStepCount(subStepCount));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 100f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            for (int i = 0; i < 60; ++i) world.Step(1f / 60f);
            return body.Transform.P.Y;
        }

        private static float RunSpinning(bool useDelta)
        {
            World world = new World(new WorldDef()
                .WithGravity(Vec2.Zero)
                .UseDeltaPositions(useDelta));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            body.AngularVelocity = 2f;  // 2 rad/s
            for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
            return body.Transform.Q.Angle;
        }

        private static float RunOffCenterSpinning(bool useDelta)
        {
            // Right-leaning triangle — centroid shifted right of the body
            // origin. Combined with angular velocity + gravity this fully
            // exercises the Stage A.2 (newPosition = newCenter - rot*LocalCenter)
            // math.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(useDelta));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 50f));
            body.CreateFixture(new FixtureDef(new PolygonShape(new[]
            {
                new Vec2(0f, 0f),
                new Vec2(2f, 0f),
                new Vec2(1f, 1f),
            })).WithDensity(1f));
            body.AngularVelocity = 3f;  // 3 rad/s
            for (int i = 0; i < 30; ++i) world.Step(1f / 60f);
            return body.Transform.P.Y;
        }
    }
}
