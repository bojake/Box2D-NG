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

        private static float RunFreeFall(bool useDelta)
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .UseDeltaPositions(useDelta));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 100f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            for (int i = 0; i < 60; ++i) world.Step(1f / 60f);
            return body.Transform.P.Y;
        }
    }
}
