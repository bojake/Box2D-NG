using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace Box2DNG.Tests
{
    [TestClass]
    public class SimdSolverTests
    {
        // These tests force the SIMD solver path on and verify the lanes give
        // equivalent results to the scalar path for the same scenarios — exercising
        // the vectorized tangent-speed and rolling-resistance code added to
        // SolveVelocityBatch and SolveVelocityBatchTwoPoint.

        private static (World scalar, World simd) BuildPair(Func<WorldDef, WorldDef> tune, Action<World> build)
        {
            WorldDef scalarDef = tune(new WorldDef().EnableContactSolverSimdPath(false));
            WorldDef simdDef = tune(new WorldDef().EnableContactSolverSimdPath(true));
            World scalar = new World(scalarDef);
            World simd = new World(simdDef);
            build(scalar);
            build(simd);
            return (scalar, simd);
        }

        [TestMethod]
        public void Simd_PyramidStack_RestsLikeScalarPath()
        {
            // A stack of boxes — generates many constraints, so the SIMD batched paths
            // engage on machines where Vector<float>.Count > 1.
            void Build(World w)
            {
                Body ground = w.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
                ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-20f, 0f), new Vec2(20f, 0f))));
                Vec2[] boxVerts = { new Vec2(-0.5f, -0.5f), new Vec2(0.5f, -0.5f), new Vec2(0.5f, 0.5f), new Vec2(-0.5f, 0.5f) };
                for (int row = 0; row < 5; ++row)
                {
                    for (int col = 0; col <= row; ++col)
                    {
                        float x = (col - row * 0.5f) * 1.05f;
                        float y = 0.5f + (5 - row) * 1.05f;
                        Body b = w.CreateBody(new BodyDef().AsDynamic().At(x, y));
                        b.CreateFixture(new FixtureDef(new PolygonShape(boxVerts)).WithDensity(1f).WithFriction(0.5f));
                    }
                }
            }

            (World scalar, World simd) = BuildPair(d => d.WithGravity(new Vec2(0f, -10f)), Build);
            for (int i = 0; i < 60; ++i)
            {
                scalar.Step(1f / 60f);
                simd.Step(1f / 60f);
            }

            // SIMD and scalar paths produce slightly different float ordering,
            // but the bulk macro-state should be very close.
            Assert.AreEqual(scalar.Bodies.Count, simd.Bodies.Count);
            for (int i = 1; i < scalar.Bodies.Count; ++i)
            {
                Vec2 ps = scalar.Bodies[i].Transform.P;
                Vec2 pd = simd.Bodies[i].Transform.P;
                Assert.AreEqual(ps.X, pd.X, 0.1f, $"body {i} X");
                Assert.AreEqual(ps.Y, pd.Y, 0.1f, $"body {i} Y");
            }
        }

        [TestMethod]
        public void Simd_TangentSpeed_DragsBoxAlongConveyor()
        {
            // With Simd on and a conveyor (tangentSpeed != 0), the lane code must
            // subtract tangentSpeedV from vt to drag the box.
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)).EnableContactSolverSimdPath(true));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f)))
                .WithFriction(1.0f)
                .WithTangentSpeed(5f));

            Body box = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0.6f));
            box.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f).WithFriction(1.0f));

            for (int s = 0; s < 60; ++s) world.Step(1f / 60f);
            float startX = box.Transform.P.X;
            for (int i = 0; i < 240; ++i) world.Step(1f / 60f);
            float dx = MathF.Abs(box.Transform.P.X - startX);
            Assert.IsTrue(dx > 0.01f, $"SIMD conveyor should drag the box. dx = {box.Transform.P.X - startX}");
        }

        [TestMethod]
        public void Simd_RollingResistance_DampsAngularVelocity()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)).EnableContactSolverSimdPath(true));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f)))
                .WithFriction(1.0f)
                .WithRollingResistance(1.0f));

            Body wheel = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0.6f));
            wheel.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f).WithFriction(1.0f).WithRollingResistance(1.0f));

            for (int settle = 0; settle < 30; ++settle) world.Step(1f / 60f);
            wheel.AngularVelocity = 20f;
            for (int i = 0; i < 120; ++i) world.Step(1f / 60f);
            Assert.IsTrue(MathF.Abs(wheel.AngularVelocity) < 18f,
                $"SIMD rolling resistance should reduce angular velocity from 20. Now: {wheel.AngularVelocity}");
        }
    }
}
