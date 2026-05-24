using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 1 of TIER4_PARITY_PLAN: pin the soft-weld behaviour. Hertz=0
    /// preserves the legacy rigid behaviour (covered by determinism tests);
    /// here we exercise the soft-spring path.
    /// </summary>
    [TestClass]
    public class SoftWeldJointTests
    {
        [TestMethod]
        public void SoftWeld_RigidByDefault_AnchorVelocitiesEqual()
        {
            // Two bodies welded at the world origin (their shared anchor). With
            // a rigid weld the velocities *at the anchor point* of each body
            // must match — `vA + cross(wA, rA) == vB + cross(wB, rB)`. The
            // bodies' linear velocities individually are NOT equal when the
            // system rotates (it does in this scenario).
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(-1f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(a, b, new Vec2(0f, 0f)));

            b.LinearVelocity = new Vec2(0f, 5f);
            world.Step(1f / 60f);

            Vec2 vA_anchor = a.LinearVelocity + Vec2.Cross(a.AngularVelocity, new Vec2(1f, 0f));
            Vec2 vB_anchor = b.LinearVelocity + Vec2.Cross(b.AngularVelocity, new Vec2(-1f, 0f));
            float relAnchorVel = (vB_anchor - vA_anchor).Length;
            Assert.IsTrue(relAnchorVel < 0.5f,
                $"Rigid weld should equalize anchor velocities. relAnchorVel={relAnchorVel}");
        }

        [TestMethod]
        public void SoftWeld_LowHertz_AllowsAnchorDrift()
        {
            // A very soft spring lets the anchor points drift apart noticeably
            // before the spring snaps them back. Contrast with the rigid case
            // where anchor velocities equalize in one step.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(-1f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(a, b, new Vec2(0f, 0f))
                .WithLinearSpring(2f, 0.1f)  // very soft, lightly damped
                .WithAngularSpring(2f, 0.1f));

            b.LinearVelocity = new Vec2(0f, 5f);
            for (int i = 0; i < 12; ++i)
            {
                world.Step(1f / 60f);
            }
            // After 0.2 s the soft spring hasn't fully equalized — anchor
            // velocities can still differ noticeably.
            Vec2 vA_anchor = a.LinearVelocity + Vec2.Cross(a.AngularVelocity, new Vec2(1f, 0f) - new Vec2(a.Transform.P.X + 1f, 0f));
            Vec2 vB_anchor = b.LinearVelocity + Vec2.Cross(b.AngularVelocity, new Vec2(-1f, 0f) - new Vec2(b.Transform.P.X - 1f, 0f));
            float relAnchorVel = (vB_anchor - vA_anchor).Length;
            Assert.IsTrue(relAnchorVel > 0.5f,
                $"Soft weld should allow residual anchor-velocity mismatch. relAnchorVel={relAnchorVel}");
        }

        [TestMethod]
        public void SoftWeld_HighHertz_BehavesNearRigid()
        {
            // At very high Hertz the soft spring asymptotically matches rigid:
            // anchor velocities should be near-equal after one step.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(-1f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(a, b, new Vec2(0f, 0f))
                .WithLinearSpring(120f, 0.7f)
                .WithAngularSpring(60f, 0.7f));

            b.LinearVelocity = new Vec2(0f, 5f);
            world.Step(1f / 60f);

            Vec2 vA_anchor = a.LinearVelocity + Vec2.Cross(a.AngularVelocity, new Vec2(1f, 0f));
            Vec2 vB_anchor = b.LinearVelocity + Vec2.Cross(b.AngularVelocity, new Vec2(-1f, 0f));
            float relAnchorVel = (vB_anchor - vA_anchor).Length;
            Assert.IsTrue(relAnchorVel < 1.5f,
                $"High-Hz soft weld should approach rigid. relAnchorVel={relAnchorVel}");
        }

        [TestMethod]
        public void SoftWeld_DampedSpring_OscillationDecaysToRest()
        {
            // A hanging body welded to a fixed anchor with a damped spring
            // should settle to rest under gravity within a few seconds. Without
            // damping it would oscillate indefinitely; with critical damping
            // (ratio=1) it converges fastest.
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            Body hanging = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            hanging.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(anchor, hanging, new Vec2(0f, 5f))
                .WithLinearSpring(10f, 1.0f)
                .WithAngularSpring(10f, 1.0f));

            // Settle for 5 seconds.
            for (int i = 0; i < 300; ++i)
            {
                world.Step(1f / 60f);
            }
            float finalSpeed = hanging.LinearVelocity.Length;
            Assert.IsTrue(finalSpeed < 0.5f, $"Damped soft weld should settle. finalSpeed={finalSpeed}");
            // Body should still be near the anchor — verifies the spring holds.
            float drift = (hanging.Transform.P - new Vec2(0f, 5f)).Length;
            Assert.IsTrue(drift < 2f, $"Damped soft weld shouldn't drift far. drift={drift}");
        }

        [TestMethod]
        public void SoftWeld_AngularSpring_DampsRotation()
        {
            // Two welded bodies with only an angular spring set. B starts
            // spinning relative to A; the angular spring damps it out.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(-1f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new PolygonShape(new[]
            {
                new Vec2(-0.5f, -0.1f), new Vec2(0.5f, -0.1f),
                new Vec2(0.5f, 0.1f), new Vec2(-0.5f, 0.1f)
            })).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(a, b, new Vec2(0f, 0f))
                .WithLinearSpring(120f, 0.7f)
                .WithAngularSpring(8f, 1.0f));

            b.AngularVelocity = 10f;
            for (int i = 0; i < 180; ++i)
            {
                world.Step(1f / 60f);
            }
            Assert.IsTrue(MathF.Abs(b.AngularVelocity) < 1f,
                $"Angular spring should damp rotation. w={b.AngularVelocity}");
        }

        [TestMethod]
        public void SoftWeld_WorldDefaultHertz_AppliedWhenJointHertzIsZero()
        {
            // Joint has no per-joint Hertz; world's JointHertz fallback applies.
            World world = new World(new WorldDef()
                .WithGravity(Vec2.Zero)
                .WithJointHertz(30f)
                .WithJointDampingRatio(0.5f));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(-1f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new WeldJointDef(a, b, new Vec2(0f, 0f)));  // no spring set

            b.LinearVelocity = new Vec2(0f, 5f);
            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }
            // World default hertz should produce a soft-spring response —
            // body should still be near the anchor.
            float drift = (b.Transform.P - new Vec2(1f, 0f)).Length;
            Assert.IsTrue(drift < 0.5f,
                $"World JointHertz default should constrain drift. drift={drift}");
        }
    }
}
