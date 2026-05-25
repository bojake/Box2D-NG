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
